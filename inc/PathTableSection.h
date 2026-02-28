#pragma once
#include "SectionState.h"
#include "MapSystem.h"


class PathTableSection
{
public:
    PathTableSection(const vector<SectionPath*>& paths, int window, int k_robust, MapSystem* MapSys);

    void remove(const SectionPath* old_path, int agent, MapSystem* MapSys);
    list<std::shared_ptr<SectionConflict> > add(const SectionPath* new_path, int agent, MapSystem* MapSys);


private:

    // Key: (Time, Section, Cell) 조합 -> Value: [이곳을 지나가는 에이전트 ID 목록]
    std::unordered_map<uint32_t, std::vector<int>> cell_table; 
    
    // SectionID -> { Time -> CountDelta (들어오면 +1, 나가면 -1) }
    std::unordered_map<int, std::map<int, int>> section_timeline;
    //unordered_map<int, list<pair<int, int> > > PT; // key: location; value: list of time-agent pair
    
    // 각 데이터가 차지할 비트 수
    static constexpr int BITS_CELL    = 4;  // Max 15
    static constexpr int BITS_SECTION = 14; // Max 16383
    // Time은 나머지 14비트 사용 (Max 16383)

    // 시프트 위치 계산
    static constexpr int SHIFT_SECTION = BITS_CELL;             // 4
    static constexpr int SHIFT_TIME    = BITS_CELL + BITS_SECTION; // 4 + 14 = 18

    // 마스크 (유효성 검사용, 선택 사항)
    static constexpr int MAX_CELL    = (1 << BITS_CELL) - 1;
    static constexpr int MAX_SECTION = (1 << BITS_SECTION) - 1;
    static constexpr int MAX_TIME    = (1 << (32 - SHIFT_TIME)) - 1;

    inline uint32_t make_cell_key(int time, int section_id, int cell_idx) const;
    
    int window;
    int k_robust;
    int num_of_agents;


};
