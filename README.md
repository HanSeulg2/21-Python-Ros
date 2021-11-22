<!--
# Robot Operating System - 미로찾기, 로봇청소기
- Language : Python
- OS : Ubuntu Linux 16.04
- Tools : PyCharm
- Open Source : Turtlebot3/gazebo
- 형성관리 : Smartgit

## 미로찾기
- BFS로 미로의 모든 경로를 탐색한다.
- 미로를 탈출한 이후 Back Tracking으로 최단 경로로 출발지점으로 복귀한다.
 
![미로찾기1](https://user-images.githubusercontent.com/71927210/129312506-04f88843-cbc2-4627-a10e-5bee1506d74a.png)
![미로찾기2](https://user-images.githubusercontent.com/71927210/129312542-0ccbbec0-425d-4c1b-ba43-f0c7c807bd50.png)
![미로찾기3](https://user-images.githubusercontent.com/71927210/129312556-d4b33e90-52b6-4e70-ac8b-af6c9d8831ca.png)
![미로찾기4](https://user-images.githubusercontent.com/71927210/129312562-50098926-12dd-40a3-8e17-a9482f8e976e.png)

## 로봇청소기
- 좌수법으로 정해진 장소의 범위를 파악한다.
- 이후 내부의 모든 장소를 방문하여 청소를 진행한다.
 
![로봇청소기2](https://user-images.githubusercontent.com/71927210/129312620-33bbf6ec-0700-4544-8b07-ab5ef068c88d.png)
![로봇청소기3](https://user-images.githubusercontent.com/71927210/129312696-1ba93533-3181-468a-8d92-429029467d50.png)
![로봇청소기5](https://user-images.githubusercontent.com/71927210/129312884-8ddf0dca-ccde-4181-bab7-e73666c43912.png)
![로봇청소기7](https://user-images.githubusercontent.com/71927210/129313009-c5d25916-c8aa-499b-93d8-733c9b20e768.png)
![로봇청소기4](https://user-images.githubusercontent.com/71927210/129312716-1c421595-ba59-48c8-a193-2e1a925ef5b5.png)
![로봇청소기6](https://user-images.githubusercontent.com/71927210/129312876-198e161c-770e-4315-96f5-d44cad94715b.png)
### 요구사항
- 미로찾기
   - ![미로찾기](https://user-images.githubusercontent.com/71927210/129307847-5ec0662e-2641-4c4f-8d6f-d9b0ba2865c1.png)
- 로봇청소기
   - ![로봇청소기](https://user-images.githubusercontent.com/71927210/129307972-943d5450-1f3e-481b-bb53-929ee8422a83.png)

### 관련기술 
- 미로찾기
   - ![미로찾기](https://user-images.githubusercontent.com/71927210/129308268-5a40c971-2312-4b83-aa89-fa41d97ea32e.png)
- 로봇청소기
   - ![로봇청소기](https://user-images.githubusercontent.com/71927210/129308326-0796ec0c-a1f2-4e9d-b918-5d1ffb863a99.png)

# 유스케이스
## 미로 찾기
![image](https://user-images.githubusercontent.com/71927210/129740802-7f9b7fff-701a-4f6e-866f-0cab72c8ab41.png)
![image](https://user-images.githubusercontent.com/71927210/129740989-cd6892c3-2bc9-446a-a85d-d360fe81ee2e.png)
![image](https://user-images.githubusercontent.com/71927210/129741137-a619c6a2-3137-4c3a-b73f-07d641f01d2e.png)
## 로봇 청소기
![image](https://user-images.githubusercontent.com/71927210/129743090-e5791d0b-4b20-453d-a53f-88309ab2c93a.png)
![image](https://user-images.githubusercontent.com/71927210/129743228-fa428416-4660-4b70-9410-c657a98d9639.png)
![image](https://user-images.githubusercontent.com/71927210/129743298-0991e8ff-6beb-440a-a2ac-66415c414568.png)
![image](https://user-images.githubusercontent.com/71927210/129743379-6b93231b-5a0f-4a3b-bb0e-96eff8ddc397.png)
![image](https://user-images.githubusercontent.com/71927210/129743440-10447d30-c2fd-46ec-95b0-4823f6bd1c6a.png)

# 시퀀스 다이어그램 
## 미로 찾기
![image](https://user-images.githubusercontent.com/71927210/129741240-36d55e48-1592-472b-ad9e-fe6bceea2830.png)
## 로봇 청소기
![image](https://user-images.githubusercontent.com/71927210/129743150-d7373e28-0ef4-4093-9594-d2f79252526e.png)
![image](https://user-images.githubusercontent.com/71927210/129743325-c3831f5e-271e-4d26-8215-dd47aa4a8ac1.png)
![image](https://user-images.githubusercontent.com/71927210/129743401-a804d869-5d24-43d1-86bd-f0dd169ae220.png)
![image](https://user-images.githubusercontent.com/71927210/129743458-9ccc4587-5d53-49e8-a06b-31ae8f77abea.png)

# 시스템 오퍼레이션
## 미로찾기
시스템 오퍼레이션 정의 | 미로 찾기 로봇은 장애 요소를 인식하고 저장한다.
-- | --
이름 | BotInfo.first_escape(self)
책임 | 로봇은 이동하는 경로에 장애 요소를 인식, 저장한다.
유형 | 시스템
참조 | 유스 케이스 : 장애 요소 인식 및 저장
주석 |  
예외조건 | 막다른 길로 인식하여 해당 좌표를 저장한다. 두 갈래 길로         인식하여 해당 좌표를 저장한다.
출력 |  
사전조건 | 로봇은 진행 방향을 변경할 수 있다.
사후조건 | 장애 요소를 발견하고 해당 좌표를 저장한다.

시스템 오퍼레이션 정의 | 미로 찾기 로봇은 주행 경로 저장한다.
-- | --
이름 | BotInfo.save_xy(self)
책임 | 로봇은 반환지점에 도착하는 동안 거친 모든 경로를 저장한다.
유형 | 시스템
참조 | 유스 케이스 : 주행 경로 저장
주석 |  
예외조건 | 이동한 모든 경로들을 저장해야 한다.
출력 |  
사전조건 | 로봇은 주행할 수 있다.
사후조건 | 로봇은 주행 경로를 저장한다.


시스템 오퍼레이션 정의 | 미로 찾기 로봇은 최단경로로 주행한다.
-- | --
이름 | BackTo.sec_escape(self)
책임 | 로봇은 주행 경로로부터 최단경로로 주행한다.
유형 | 시스템
참조 | 유스 케이스 : 최단 경로 계산
주석 |  
예외조건 | 로봇의 주행 좌표를 전달받지 못할 경우 정지시킨다.
출력 |  
사전조건 | 로봇의 주행 경로를 전달받는다.
사후조건 | 로봇은 최단경로로 시작지점으로 이동한다.

## 로봇 청소기

시스템 오퍼레이션 정의 | 로봇 청소기는 현재 있는 위치에서 주행을 시작한다.
-- | --
이름 | FollowingWall.following(self)
책임 | 출발 지점에서 시작한다.
유형 | 시스템
참조 | 유스 케이스 : 출발 지점에서 주행힌다.
주석 |  
예외조건 | 로봇이 벽면을 마주보고 있어야 하며 orientation값을 변경하여      회전할 수 있도록 한다.
출력 | 로봇 청소기 시작을 알린다.
사전조건 | 로봇 청소기는 대기 장소에 존재해야 한다.
사후조건 | 로봇 청소기는 주행을 위해 최대 50m/sec의 속도로 주행을         시작해야 한다.


시스템 오퍼레이션 정의 | 로봇 청소기는 주행하며 주어진 면적을 청소한다.
-- | --
이름 | VacuumClean.set_angleself)
책임 | 주어진 공간을 청소한다.
유형 | 시스템
참조 | 유스 케이스 : 로봇 청소기 청소
주석 |  
예외조건 | 최대한 많은 면적을 청소해야 한다.
출력 | 현재 터틀 봇의 좌표와 면적에 대한 정보를 알려준다.
사전조건 | 청소를 위한 주어진 공간의 정보가 필요하다
사후조건 | 1. 청소한 영역에 대한 정보를 알고 있다. 2. 최대한 많은 면적을 청소했다.


시스템 오퍼레이션 정의 | 로봇 청소기는 장애물과 벽에 충돌을 회피한다..
-- | --
이름 | FollowingWall.following(self),  VacuumClean.set_angleself)
책임 | 충돌을 감지하여 회피한다.
유형 | 시스템
참조 | 유스 케이스 : 로봇 청소기는 충돌 회피
주석 |  
예외조건 | 벽과 장애물의 충돌이 발생할 경우 시스템을 종료시킨다.
출력 | 현재 좌표와 리스트에 저장된 각 꼭짓점 좌표를 알려준다.
사전조건 | 로봇은 장애물과 벽에 정보를 알고 있어야 한다.
사후조건 | 충돌 회피 후 청소 기능을 재수행한다.


시스템 오퍼레이션 정의 | 로봇 청소기는 청소 완료 후 대기장소로 이동한다.
-- | --
이름 | VacuumClean.only_turn()
책임 | 로봇 청소기는 대기장소로 이동한다.
유형 | 시스템
참조 | 유스 케이스 : 대기장소 이동
주석 |  
예외조건 |  
출력 | 로봇 청소기의 종료, 소요 시간과 면적을 알려준다.
사전조건 | 로봇 청소기는 청소가 완료된 상태여야 한다.
사후조건 | 대기 장소로 이동완료 후 시스템을 종료시킨다.

# 일정 계획 평가 
## 미로찾기
Activity No. | 소작업 | 선행 작업 | 예상 소요기간(일) | 실제 소요기간(일)
-- | -- | -- | -- | --
F | Maze 주행 초기 설정 구현 | C, D, E | 2 | 1
G | Maze 주행 초기 설정 테스팅/수정 | F | 2 | 2
H | Maze 장애요소 인식 및 저장 기능 구현 | G | 7 | 6
I | Maze 장애요소 인식 및 저장 기능 테스팅/수정 | H | 2 | 2
J | Maze 주행경로 저장 및 최단경로 계산 구현 | G | 10 | 9
K | Maze 경로 저장, 최단경로 계산 테스팅/수정 | J | 3 | 4
L | Maze 최종 종합 및 테스팅 | I, K | 3 | 4

## 로봇청소기
Activity No. | 소작업 | 선행 작업 | 예상 소요기간(일) | 실제 소요기간(일)
-- | -- | -- | -- | --
M | vacuum 초기 상태구현 | L | 3 | 1
N | vacuum 초기 상태 기능 테스팅/수정 | M | 2 | 1
O | vacuum 청소 기능 구현 | N | 7 | 5
P | vacuum 청소 기능 테스팅/수정 | O | 3 | 3
Q | vacuum 사용자 장애물 위치설정 기능 구현 | N | 5 | 1
R | vacuum 사용자 장애물 위치설정 테스팅/수정 | Q | 3 | 1
S | vacuum 충돌 회피 기능 구현 | N | 7 | 7
T | vacuum 충돌 회피 기능 테스팅/수정 | S | 3 | 3
U | vacuum 대기장소 이동 기능 구현 | N | 4 | 6
V | vacuum 대기장소 이동 기능 테스팅/수정 | U | 2 | 3
W | vacuum 최종 종합 및 테스팅 | P | 5 | 5
X | vacuum 최종 종합 테스팅 | W | 5 | 5
Y | vacuum 결과 보고서 작성 및 최종 발표 준비 | X | 6 | 3
