import matplotlib.pyplot as plt
import time
import cvxpy
import math
import numpy as np
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


from lib.angle import angle_mod

from lib import cubic_spline_planner

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [vel, omega]
T = 5  # horizon length

# mpc parameters

R = np.diag([0.1, 0.001])  # input cost matrix
#R[0,0] → 속도 입력 비용
#R[1,1] → 각속도 입력 비용

Rd = np.diag([0.1, 0.001])  # input difference cost matrix
#Rd[0,0] → 속도 변화 비용
#Rd[1,1] → 각속도 변화 비용

Q = np.diag([1, 1, 1, 1])  # state cost matrix
#Q[0,0] (x 위치 오차)
#Q[1,1] (y 위치 오차)
#Q[2,2] (속도 오차)
#Q[3,3] (yaw 오차)
#위치 오차의 비용을 증가시키면, 차량이 경로에서 벗어나는 것을 더 강하게 패널티로 부과

Qf = Q  # state final matrix
GOAL_DIS = 5.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3

  # Max iteration
DU_TH = 0.1  # iteration finish param

N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 2  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.5  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 1  # [m]

TARGET_SPEED = 2  # 목표 속도 [m/s]
MAX_SPEED = 2  # 최대 선속도 [m/s]
MIN_SPEED = -2  # 최소 선속도 [m/s]
MAX_ACCEL = 1.5  # 최대 선가속도 [m/ss]

MAX_OMEGA = 2.5  # 최대 각속도 [rad/s]
MAX_ALPHA = 3.0 # 최대 각가속도 [rad/ss]

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def pi_2_pi(angle):
    return angle_mod(angle)

def get_linear_model_matrix(v, theta, omega):
    A = np.eye(NX)  # 단위 행렬로 초기화

    # A행렬 : 
    # 입력이 없는 경우 즉, 차량이 현재 속도와 방향을 유지하면서 앞으로 나아갈 때 
    # 상태가 어떻게 변하는지 결정해주는 행렬
    A[0, 2] = DT * np.cos(theta)  # x 방향 변화
    A[1, 2] = DT * np.sin(theta)  # y 방향 변화
    A[3, 3] = 1.0  # yaw 업데이트

    # B행렬 : 차량이 입력을 받았을 때, 상태가 어떻게 변하는지를 결정하는 행렬
    B = np.zeros((NX, NU))
    B[0, 0] = DT * np.cos(theta)  # 가속도 입력에 대한 x 변화
    B[1, 0] = DT * np.sin(theta)  # 가속도 입력에 대한 y 변화
    B[2, 0] = 1  # 속도 업데이트
    B[3, 1] = DT  # yaw 업데이트 (각속도 입력)

    #DD 모델에서는 선형화했을 때 오차가 거의 없기 때문에 C 행렬을 생략해도 무방
    C = np.zeros(NX)

    return A, B, C

def plot_car(x, y, yaw, truckcolor="-k"):

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    rr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")

def update_state(state, v_input, omega):

    # 속도 업데이트
    state.v += v_input
    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    # 각속도 제한
    if omega > MAX_OMEGA:
        omega = MAX_OMEGA
    elif omega < -MAX_OMEGA:
        omega = -MAX_OMEGA

    # 상태 업데이트 (차동구동 모델)
    state.x += state.v * math.cos(state.yaw) * DT
    state.y += state.v * math.sin(state.yaw) * DT
    state.yaw += omega * DT
    
    return state

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def predict_motion(x0, v_input, oomega, xref):
    """
    주어진 초기 상태와 입력 시퀀스를 기반으로 예측된 궤적을 계산하는 함수
    :param x0: 초기 상태 [x, y, v, yaw]
    :param v_input: 속도 시퀀스
    :param oomega: 각속도 시퀀스
    :param xref: 참조 궤적
    :return: 예측된 궤적 xbar
    """
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (vi, omega, i) in zip(v_input, oomega, range(1, T + 1)):
        state = update_state(state, vi, omega)  # 입력을 속도(v_input)와 각속도(oomega)로 사용
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar

def iterative_linear_mpc_control(xref, x0, dref, v_input, oomega):
    """
    속도(v_input)와 각속도(oomega)를 사용하는 DD 모델용 반복적 선형 MPC 컨트롤러
    :param xref: 참조 궤적 (NX x (T+1))
    :param x0: 초기 상태 [x, y, v, yaw]
    :param dref: 참조 입력 (사용하지 않음)
    :param v_input: 이전 속도 입력 (길이 T)
    :param oomega: 이전 각속도 입력 (길이 T)
    :return: 최적 속도 입력, 각속도 입력, 예측 궤적 (ox, oy, oyaw, ov)
    """

    # 초기 입력 설정 (numpy 배열 사용)
    if v_input is None or oomega is None:
        v_input = np.zeros(T)
        oomega = np.zeros(T)
        #oomega = np.ones(T) * (xref[3, 1] - x0[3]) / DT  # 초기 yaw 변화율 반영

    # 예측 결과 초기화 (DD 모델에 맞게 설정)
    ox, oy, oyaw, ov = np.zeros(T+1), np.zeros(T+1), np.zeros(T+1), np.zeros(T+1)

    for i in range(MAX_ITER):
        # 예측 궤적 생성
        xbar = predict_motion(x0, v_input, oomega, xref)

        # 이전 입력 저장
        p_v_input, p_oomega = v_input.copy(), oomega.copy()

        # 선형 MPC 컨트롤 계산 (DD 모델에 맞게 수정 필요)
        v_input, oomega, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, v_input, oomega)

        # 입력 변화량 계산 및 수렴 판별
        du = np.sum(np.abs(v_input - p_v_input)) + np.sum(np.abs(oomega - p_oomega))
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return v_input, oomega, ox, oy, oyaw, ov

def linear_mpc_control(xref, xbar, x0, v_input, oomega):
    """
    속도(v_input)와 각속도(oomega)를 사용하는 DD 모델용 선형 MPC 컨트롤러
    :param xref: 참조 궤적
    :param xbar: 예측된 궤적 (운용점)
    :param x0: 초기 상태 [x, y, v, yaw]
    :param v_input: 이전 속도 입력
    :param oomega: 이전 각속도 입력
    :return: 최적 입력 및 예측 궤적 (ox, oy, oyaw, ov)
    """

    x = cvxpy.Variable((NX, T + 1))  # 상태 변수 [x, y, v, yaw]
    u = cvxpy.Variable((NU, T))  # 입력 변수 [속도(v), 각속도(omega)]

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)  # 입력 크기 최소화 비용

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)  # 상태 오차 최소화 비용

        # DD 모델에 맞는 선형 모델 행렬 계산 (속도와 각속도를 입력으로 사용)
        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], oomega[t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        # 입력 변화량 제한 (속도와 각속도 변화)
        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_ALPHA * DT]  # 각속도 변화 제한
            constraints += [cvxpy.abs(u[0, t + 1] - u[0, t]) <= MAX_ACCEL * DT]  # 속도 변화 제한

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)  # 최종 상태 오차 비용

    # 상태 및 입력 제약 조건
    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED, x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_SPEED]  # 속도 입력 제약
    constraints += [cvxpy.abs(u[1, :]) <= MAX_OMEGA]  # 각속도 입력 제약

    # 최적화 문제 풀기
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    # 최적해 확인 및 반환
    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        v_input = get_nparray_from_matrix(u.value[0, :])  # 속도 입력
        oomega = get_nparray_from_matrix(u.value[1, :])  # 각속도 입력
    else:
        print("Error: Cannot solve MPC.. Using previous inputs.")
        v_input, oomega = v_input, oomega
        ox, oy, oyaw, ov = None, None, None, None  # 예측 궤적은 리셋

    return v_input, oomega, ox, oy, oyaw, ov

def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))  # 각속도(omega) 참조 값
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = np.arctan2(np.sin(cyaw[ind]), np.cos(cyaw[ind]))  # ✅ yaw 값 보정
    dref[0, 0] = 0.0  # 각속도 기본값 설정

    travel = 0.0
    for i in range(1, T + 1):  # ✅ i=0은 위에서 초기화되었으므로 1부터 시작
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = np.arctan2(np.sin(cyaw[ind + dind]), np.cos(cyaw[ind + dind]))  # ✅ yaw 보정
            dref[0, i] = np.arctan2(np.sin(cyaw[ind + dind] - cyaw[ind]), np.cos(cyaw[ind + dind] - cyaw[ind])) / DT  # ✅ 각속도 보정
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = np.arctan2(np.sin(cyaw[ncourse - 1]), np.cos(cyaw[ncourse - 1]))  # ✅ yaw 보정
            dref[0, i] = 0.0

    return xref, ind, dref

def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False

def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cyaw: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]
    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    oomega, oa = None, None

    cyaw = smooth_yaw(cyaw)

    # 실시간 시뮬레이션을 위한 인터랙티브 모드 활성화
    plt.ion()
    
    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        # MPC 제어
        oa, oomega, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, oomega)

        omega, ai = 0.0, 0.0
        if oomega is not None:
            omega, ai = oomega[0], oa[0]
            state = update_state(state, ai, omega)

        time = time + DT

        # 경로와 차량 위치를 기록
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(omega)
        a.append(ai)

        # 목표에 도달했는지 확인
        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        # 실시간으로 애니메이션 업데이트
        if show_animation:  # pragma: no cover
            plt.cla()  # 이전 그래프 지우기
            # 키 입력 이벤트로 시뮬레이션 종료 (esc 키)
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw)  # 차량 그리기
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.01)  # 0.01초 대기 후 화면 갱신

    plt.ioff()  # 인터랙티브 모드 종료
    plt.show()  # 마지막에 그래프 출력
    
    return t, x, y, yaw, v, d, a

def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    DD 모델에 맞는 속도 프로파일 계산 함수

    cx, cy: 경로의 x, y 좌표
    cyaw: 경로의 yaw 값
    target_speed: 목표 속도
    """
    speed_profile = [target_speed] * len(cx)  # 초기 속도 설정
    direction = 1.0  # 기본 전방

    # 각도 차이에 따라 속도 조절
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)  # 경로의 방향

        # 경로 방향과 현재 yaw 간의 차이
        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))

            # 경로 방향과 각도의 차이가 45도 이상일 경우 속도를 낮춤
            if dangle >= math.pi / 4.0:
                direction = -1.0  # 후방
            else:
                direction = 1.0  # 전방

        # 전방일 경우에는 목표 속도 유지, 후방일 경우 속도 반대 설정
        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed

    # 목표 지점에서는 속도 0
    speed_profile[-1] = 0.0

    return speed_profile

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck

def get_sin_course(dl, length=50.0, amplitude=5.0):
    """
    Sine 형태의 경로 생성
    :param dl: 경로의 샘플링 간격 (단위: m)
    :param length: 경로의 총 길이
    :param amplitude: 사인 곡선의 진폭
    :return: x, y, yaw, curvature
    """
    # 경로 길이에 맞는 x 값 생성 (0부터 length까지)
    x = np.arange(0, length, dl)
    # y 값은 sin 함수로 계산
    y = amplitude * np.sin(x / length * 2 * np.pi)
    
    # yaw 값은 각도 계산 (tan의 아크를 이용해 경로의 기울기 추정)
    yaw = np.arctan(np.gradient(y, x))  # y에 대한 x의 변화율을 기반으로 기울기 계산
    
    # 곡률 계산 (2차 미분을 이용한 계산)
    curvature = np.gradient(yaw, x)
    
    return x, y, yaw, curvature

def get_course(dl):
    ax = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 5.0, -10.0, 15.0, -20.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def main():
    print(__file__ + " start!!")
    start = time.time()

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_switch_back_course(dl)
    #cx, cy, cyaw, ck = get_sin_course(dl)
    #cx, cy, cyaw, ck = get_course(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=sp[0])  # 속도로 초기 상태 설정

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    elapsed_time = time.time() - start
    print(f"calc time:{elapsed_time:.6f} [sec]")

    if show_animation:  # pragma: no cover
        plt.close("all")

        # 선가속도와 각가속도 계산
        a_accel = np.diff(a) / DT  # Linear acceleration (m/s²)
        d_accel = np.diff(d) / DT  # Angular acceleration (rad/s²)
        t_accel = t[:-1]  # 시간 배열 조정 (diff 사용 시 길이가 하나 줄어듦)

        # 📌 경로 그래프 (별도 창)
        fig1, ax1 = plt.subplots(figsize=(8, 6))
        ax1.plot(cx, cy, "-r", label="spline")
        ax1.plot(x, y, "-g", label="tracking")
        ax1.grid(True)
        ax1.axis("equal")
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.legend()
        ax1.set_title("Path Tracking")
        plt.show(block=False)  # 창을 유지하면서 다음 그래프 실행

        # 📌 속도 & 가속도 (한 창에 2x2)
        fig2, axs = plt.subplots(2, 2, figsize=(10, 8))

        # 선속도 (Linear Speed)
        axs[0, 0].plot(t, v, "-r", label="Linear Speed")
        axs[0, 0].grid(True)
        axs[0, 0].set_xlabel("Time [s]")
        axs[0, 0].set_ylabel("Speed [m/s]")
        axs[0, 0].legend()
        axs[0, 0].set_title("Linear Speed Profile")

        # 각속도 (Angular Speed)
        axs[0, 1].plot(t, d, "-b", label="Angular Speed")
        axs[0, 1].grid(True)
        axs[0, 1].set_xlabel("Time [s]")
        axs[0, 1].set_ylabel("Angular Speed [rad/s]")
        axs[0, 1].legend()
        axs[0, 1].set_title("Angular Speed Profile")

        # 선가속도 (Linear Acceleration)
        axs[1, 0].plot(t_accel, a_accel, "-g", label="Linear Acceleration")
        axs[1, 0].grid(True)
        axs[1, 0].set_xlabel("Time [s]")
        axs[1, 0].set_ylabel("Acceleration [m/s²]")
        axs[1, 0].legend()
        axs[1, 0].set_title("Linear Acceleration Profile")

        # 각가속도 (Angular Acceleration)
        axs[1, 1].plot(t_accel, d_accel, "-m", label="Angular Acceleration")
        axs[1, 1].grid(True)
        axs[1, 1].set_xlabel("Time [s]")
        axs[1, 1].set_ylabel("Angular Acceleration [rad/s²]")
        axs[1, 1].legend()
        axs[1, 1].set_title("Angular Acceleration Profile")

        plt.tight_layout()
        plt.show()

        
if __name__ == '__main__':
    main()
