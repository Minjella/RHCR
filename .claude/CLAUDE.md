# project
논문을 쓰기 위한 Project고 baseline을 RHCR (PBS-SIPP)를 사용하기 위해 기존의 RHCR 코드에서 내 코드를 추가했어 문제는 Lifelong Multi-Agent Path Finding이야

# target
목표는 코드를 최적화 시키고 성공률이 모든 상황에 대해서 100프로가 나올 수 있게 만드는거야

# IDEA
기존의 sorting map을 section화 시켜서 node 증가량이나 이런 부분에서 이득을 봐서 계산 시간이나 계산량을 줄일 수 있지 않을까?
기존의 sorting map이 templete을 가지고 반복되는 구조로 되어있어서 templete을 사전에 미리 저장한 이후에 그 templete 정보를 가지고 section 단위의 경로 계획

# ADD or Modify File (My File)
inc/BasicSystem.h
inc/MAPFSolver.h
inc/PathTableSection.h
inc/PBSNodeSection.h
inc/PBSSection.h
inc/ReservationSection.h
inc/Section.h
inc/SectionState.h
inc/SIPPSection.h
inc/SortingSystem.h
src/BasicSystem.cpp
src/MAPFSolver.cpp
src/PathTableSection.cpp
src/PBSNodeSection.cpp
src/PBSSection.cpp
src/ReservationSection.cpp
src/Section.cpp
src/SectionState.cpp
src/SIPPSection.cpp
src/SortingSystem.cpp

# Validation
코드를 수정하고 build 하기 전에 다시 한 번 코드에 문제점은 없는지 확인해줘

# 현재 진행 상황
지금은 agent 500대 정도가 되면 no solution이 나오는 경우가 있는데 그 경우를 debuging하고 수정하는 작업을 하고 있어

# 코드 실행
rm -rf build
cmake -S . -B build 
cmake --build build -j
./lifelong -m maps/sorting_map.grid --scenario=SORTING --simulation_window=5 --planning_window=10 --solver=PBS --simulation_time=40 --seed=7 -k 500