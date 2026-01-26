#pragma once
#include <vector>
#include <iostream>
#include <functional> // for std::hash
#include "common.h"   // 필요 시 포함

struct SectionState
{
    int section_id;             // 현재 섹션 ID
    int start_index;            // 섹션 진입 위치 (Grid Index)
    int exit_index;             // 섹션 진출 예정 위치 (없으면 -1)
    int timestep;               // start_index에 도착한 시간
    int goal_index;             // goal location이 포함되어있다면 index, (없다면 -1)
    std::vector<int> wait_list; // 섹션 내 대기 정보 (대기한 index list)

    // 1. Wait 함수: "진입 시간을 1 늦춤" (즉, 이전 섹션에서 1틱 더 있다가 옴)
    // 주의: 만약 '섹션 내부에서의 대기'를 의미한다면 wait_list에 추가하는 별도 함수가 필요할 수 있습니다.
    SectionState wait(int current_idx) const;

    struct Hasher
    {
        std::size_t operator()(const SectionState& s) const
        {
            size_t id_hash = std::hash<int>()(s.section_id);
            size_t start_hash = std::hash<int>()(s.start_index);
            size_t exit_hash = std::hash<int>()(s.exit_index);
            size_t time_hash = std::hash<int>()(s.timestep);
            size_t goal_hash = std::hash<int>()(s.goal_index);
            
            // wait_list도 해시에 포함해야 정확하지만, 비용이 큽니다.
            // 우선 list의 사이즈 정도만 XOR 하거나, 필요 시 전체 순회해야 합니다.
            size_t wait_hash = std::hash<size_t>()(s.wait_list.size());

            return (time_hash ^ (id_hash << 1) ^ (start_hash << 2) ^ (exit_hash << 3) ^ (goal_hash << 4) ^ (wait_hash << 5));
        }
    };

    void operator = (const SectionState& other)
    {
        section_id = other.section_id;
        start_index = other.start_index;
        exit_index = other.exit_index;
        timestep = other.timestep;
        goal_index = other.goal_index;
        wait_list = other.wait_list; // 벡터 복사 발생
    }

    bool operator == (const SectionState& other) const
    {
        // vector 비교까지 수행 (내용이 같아야 같은 상태)
        return section_id == other.section_id &&
               start_index == other.start_index &&
               exit_index == other.exit_index &&
               timestep == other.timestep &&
               goal_index == other.goal_index &&
               wait_list == other.wait_list;
    }

    bool operator != (const SectionState& other) const
    {
        return !(*this == other);
    }

    // 생성자
    SectionState() 
        : section_id(-1), start_index(-1), exit_index(-1), timestep(-1), goal_index(-1) {}

    // 기본 생성 (wait_list는 빈 상태)
    SectionState(int section_id, int start_index, int exit_index, int timestep)
        : section_id(section_id), start_index(start_index), 
          exit_index(exit_index), timestep(timestep), goal_index(-1) {}

    // 전체 생성
    SectionState(int section_id, int start_index, int exit_index, int timestep, int goal_index, const std::vector<int>& waits)
        : section_id(section_id), start_index(start_index), 
          exit_index(exit_index), timestep(timestep), goal_index(goal_index), wait_list(waits) {}
};

// 출력 연산자 정의
std::ostream & operator << (std::ostream &out, const SectionState &s);

typedef std::vector<SectionState> SectionPath;