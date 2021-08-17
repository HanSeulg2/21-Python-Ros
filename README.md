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
# 시퀀스 다이어그램 
## 미로 찾기
![image](https://user-images.githubusercontent.com/71927210/129741186-29701499-5ea8-4556-82b5-5560e1fba5d6.png)
![image](https://user-images.githubusercontent.com/71927210/129741240-36d55e48-1592-472b-ad9e-fe6bceea2830.png)

# 시스템 오퍼레이션
## 미로 찾기
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


