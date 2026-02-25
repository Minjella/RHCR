#include "Section.h"
#include <queue>
#include <algorithm>
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <iomanip>

SectionTemplate::SectionTemplate(SectionType t, int cap) 
    : type(t), capacity(cap), adj(9), cells(9) // adj 9개로 초기화
{
    path_table.assign(9, vector<vector<int>>(9));
    internal_dist_matrix.assign(9, vector<int>(9, std::numeric_limits<int>::max()));
    entry_cells_by_dir.assign(4, -1);
    exit_cells_by_dir.assign(4, -1);

    setup_connectivity(); 
    precompute_all();
}

void SectionTemplate::setup_connectivity() {
    // 0 1 2
    // 3 4 5
    // 6 7 8
    const int DIR_EAST = 0; // x + 1
    const int DIR_SOUTH = 1; // y - 1
    const int DIR_WEST = 2; // x - 1
    const int DIR_NORTH = 3; // y + 1

    if (type == SectionType::Eject_CW) {
        width = 3;
        height = 3;

        adj[0].push_back(1); 
        adj[1].push_back(2); 
        adj[2].push_back(5);
        adj[5].push_back(8); 
        adj[8].push_back(7); 
        adj[7].push_back(6);
        adj[6].push_back(3); 
        adj[3].push_back(0);

        entries = {0, 2, 6, 8};
        exits = {0, 2, 6, 8};

        entry_cells_by_dir[DIR_EAST] = 8;
        entry_cells_by_dir[DIR_NORTH] = 2;
        entry_cells_by_dir[DIR_SOUTH] = 6;
        entry_cells_by_dir[DIR_WEST] = 0;

        exit_cells_by_dir[DIR_EAST] = 2;
        exit_cells_by_dir[DIR_NORTH] = 0;
        exit_cells_by_dir[DIR_SOUTH] = 8;
        exit_cells_by_dir[DIR_WEST] = 6;
    }
    else if (type == SectionType::Eject_CCW) {
        width = 3;
        height = 3;

        adj[0].push_back(3); 
        adj[3].push_back(6); 
        adj[6].push_back(7);
        adj[7].push_back(8); 
        adj[8].push_back(5); 
        adj[5].push_back(2);
        adj[2].push_back(1); 
        adj[1].push_back(0);

        entries = {0, 2, 6, 8};
        exits = {0, 2, 6, 8};

        entry_cells_by_dir[DIR_EAST] = 2;
        entry_cells_by_dir[DIR_NORTH] = 0;
        entry_cells_by_dir[DIR_SOUTH] = 8;
        entry_cells_by_dir[DIR_WEST] = 6;

        exit_cells_by_dir[DIR_EAST] = 8;
        exit_cells_by_dir[DIR_NORTH] = 2;
        exit_cells_by_dir[DIR_SOUTH] = 6;
        exit_cells_by_dir[DIR_WEST] = 0;
    }
    else if (type == SectionType::Eject_UP) {
        width = 3;
        height = 3;

        adj[2].push_back(1); 
        adj[1].push_back(0); 
        adj[2].push_back(5);
        adj[5].push_back(8); 
        adj[6].push_back(7); 
        adj[7].push_back(8);
        adj[6].push_back(3); 
        adj[3].push_back(0);

        entries = {2, 6};
        exits = {0, 8};

        entry_cells_by_dir[DIR_EAST] = 2;
        entry_cells_by_dir[DIR_NORTH] = 2;
        entry_cells_by_dir[DIR_SOUTH] = 6;
        entry_cells_by_dir[DIR_WEST] = 6;

        exit_cells_by_dir[DIR_EAST] = 8;
        exit_cells_by_dir[DIR_NORTH] = 0;
        exit_cells_by_dir[DIR_SOUTH] = 8;
        exit_cells_by_dir[DIR_WEST] = 0;
    }
    else if (type == SectionType::Eject_DOWN) {
        width = 3;
        height = 3;

        adj[0].push_back(1); 
        adj[1].push_back(2); 
        adj[0].push_back(3);
        adj[3].push_back(6); 
        adj[8].push_back(7); 
        adj[7].push_back(6);
        adj[8].push_back(5); 
        adj[5].push_back(2);

        entries = {0, 8};
        exits = {2, 6};

        entry_cells_by_dir[DIR_EAST] = 8;
        entry_cells_by_dir[DIR_NORTH] = 0;
        entry_cells_by_dir[DIR_SOUTH] = 8;
        entry_cells_by_dir[DIR_WEST] = 0;

        exit_cells_by_dir[DIR_EAST] = 2;
        exit_cells_by_dir[DIR_NORTH] = 2;
        exit_cells_by_dir[DIR_SOUTH] = 6;
        exit_cells_by_dir[DIR_WEST] = 6;
    }
    else if (type == SectionType::Induct_C_BOTTOM) {
        width = 3;
        height = 2;

        adj[3].push_back(4); 
        adj[4].push_back(5); 
        adj[5].push_back(8);
        adj[8].push_back(7); 
        adj[7].push_back(6); 
        adj[6].push_back(3);

        entries = {3, 5};
        exits = {3, 5};

        entry_cells_by_dir[DIR_WEST] = 3;
        entry_cells_by_dir[DIR_NORTH] = 5;

        exit_cells_by_dir[DIR_NORTH] = 3;
        exit_cells_by_dir[DIR_EAST] = 5;

        // adj[0].push_back(1); 
        // adj[1].push_back(2); 
        // adj[2].push_back(5);
        // adj[5].push_back(4); 
        // adj[4].push_back(3); 
        // adj[3].push_back(0);

        // entries = {0, 2};
        // exits = {0, 2};

        // entry_cells_by_dir[DIR_WEST] = 0;
        // entry_cells_by_dir[DIR_NORTH] = 2;

        // exit_cells_by_dir[DIR_NORTH] = 0;
        // exit_cells_by_dir[DIR_EAST] = 2;
    }
    else if (type == SectionType::Induct_N_BOTTOM) {
        width = 3;
        height = 2;

        adj[3].push_back(6); 
        adj[6].push_back(7); 
        adj[7].push_back(8);
        adj[8].push_back(5); 
        adj[3].push_back(4); 
        adj[4].push_back(5);

        entries = {3};
        exits = {5};

        entry_cells_by_dir[DIR_WEST] = 3;
        entry_cells_by_dir[DIR_NORTH] = 3;

        exit_cells_by_dir[DIR_NORTH] = 5;
        exit_cells_by_dir[DIR_EAST] = 5;

        // adj[0].push_back(3); 
        // adj[3].push_back(4); 
        // adj[4].push_back(5);
        // adj[5].push_back(2); 
        // adj[0].push_back(1); 
        // adj[1].push_back(2);

        // entries = {0};
        // exits = {2};

        // entry_cells_by_dir[DIR_WEST] = 0;
        // entry_cells_by_dir[DIR_NORTH] = 0;

        // exit_cells_by_dir[DIR_NORTH] = 2;
        // exit_cells_by_dir[DIR_EAST] = 2;
    }
    else if (type == SectionType::Induct_C_TOP) {
        width = 3;
        height = 2;

        adj[3].push_back(4); 
        adj[4].push_back(5); 
        adj[5].push_back(8);
        adj[8].push_back(7); 
        adj[7].push_back(6); 
        adj[6].push_back(3);

        entries = {6, 8};
        exits = {6, 8};

        entry_cells_by_dir[DIR_EAST] = 8;
        entry_cells_by_dir[DIR_SOUTH] = 6;

        exit_cells_by_dir[DIR_SOUTH] = 8;
        exit_cells_by_dir[DIR_WEST] = 6;

    //     adj[0].push_back(1); 
    //     adj[1].push_back(2); 
    //     adj[2].push_back(5);
    //     adj[5].push_back(4); 
    //     adj[4].push_back(3); 
    //     adj[3].push_back(0);

    //     entries = {3, 5};
    //     exits = {3, 5};

    //     entry_cells_by_dir[DIR_EAST] = 5;
    //     entry_cells_by_dir[DIR_SOUTH] = 3;

    //     exit_cells_by_dir[DIR_SOUTH] = 5;
    //     exit_cells_by_dir[DIR_WEST] = 3;
    }
    else if (type == SectionType::Induct_N_TOP) {
        width = 3;
        height = 2;

        adj[3].push_back(6); 
        adj[8].push_back(7); 
        adj[7].push_back(6);
        adj[8].push_back(5); 
        adj[5].push_back(4); 
        adj[4].push_back(3);

        entries = {8};
        exits = {6};

        entry_cells_by_dir[DIR_EAST] = 8;
        entry_cells_by_dir[DIR_SOUTH] = 8;

        exit_cells_by_dir[DIR_SOUTH] = 6;
        exit_cells_by_dir[DIR_WEST] = 6;

        // adj[0].push_back(3); 
        // adj[5].push_back(4); 
        // adj[4].push_back(3);
        // adj[5].push_back(2); 
        // adj[2].push_back(1); 
        // adj[1].push_back(0);

        // entries = {5};
        // exits = {3};

        // entry_cells_by_dir[DIR_EAST] = 5;
        // entry_cells_by_dir[DIR_SOUTH] = 5;

        // exit_cells_by_dir[DIR_SOUTH] = 3;
        // exit_cells_by_dir[DIR_WEST] = 3;
    }
    else if (type == SectionType::Travel_LEFT_BOTTOM) {
        width = 1;
        height = 3;

        adj[0].push_back(3); 
        adj[3].push_back(6); 

        entries = {0};
        exits = {6};

        entry_cells_by_dir[DIR_EAST] = 0;
        entry_cells_by_dir[DIR_NORTH] = 0;

        exit_cells_by_dir[DIR_SOUTH] = 6;
        exit_cells_by_dir[DIR_EAST] = 6;
    }
    else if (type == SectionType::Travel_LEFT_TOP) {
        width = 1;
        height = 3;

        adj[0].push_back(3); 
        adj[3].push_back(6);

        entries = {0, 6};
        exits = {0, 6};

        entry_cells_by_dir[DIR_EAST] = 6;
        entry_cells_by_dir[DIR_NORTH] = 0;

        exit_cells_by_dir[DIR_SOUTH] = 6;
        exit_cells_by_dir[DIR_EAST] = 0;
    }
    else if (type == SectionType::Travel_RIGHT_BOTTOM) {
        width = 1;
        height = 3;

        adj[6].push_back(3); 
        adj[3].push_back(0); 

        entries = {6};
        exits = {0};

        entry_cells_by_dir[DIR_WEST] = 6;
        entry_cells_by_dir[DIR_SOUTH] = 6;

        exit_cells_by_dir[DIR_NORTH] = 0;
        exit_cells_by_dir[DIR_WEST] = 0;
    }
    else if (type == SectionType::Travel_RIGHT_TOP) {
        width = 1;
        height = 3;

        adj[6].push_back(3); 
        adj[3].push_back(0); 

        entries = {0, 6};
        exits = {0, 6};

        entry_cells_by_dir[DIR_WEST] = 0;
        entry_cells_by_dir[DIR_SOUTH] = 6;


        exit_cells_by_dir[DIR_NORTH] = 0;
        exit_cells_by_dir[DIR_WEST] = 6;
    }
    else if (type == SectionType::Corner_RIGHT_BOTTOM) {
        width = 1;
        height = 2;

        entries = {3};
        exits = {3};

        entry_cells_by_dir[DIR_WEST] = 3;

        exit_cells_by_dir[DIR_NORTH] = 3;

        // entries = {0};
        // exits = {0};

        // entry_cells_by_dir[DIR_WEST] = 0;

        // exit_cells_by_dir[DIR_NORTH] = 0;
    }
    else if (type == SectionType::Corner_LEFT_BOTTOM) {
        width = 1;
        height = 2;

        entries = {3};
        exits = {3};

        entry_cells_by_dir[DIR_NORTH] = 3;

        exit_cells_by_dir[DIR_EAST] = 3;

        // entries = {0};
        // exits = {0};

        // entry_cells_by_dir[DIR_NORTH] = 0;

        // exit_cells_by_dir[DIR_EAST] = 0;
    }
    else if (type == SectionType::Corner_RIGHT_TOP) {
        width = 1;
        height = 2;

        entries = {6};
        exits = {6};

        entry_cells_by_dir[DIR_SOUTH] = 6;

        exit_cells_by_dir[DIR_WEST] = 6;

        // entries = {3};
        // exits = {3};

        // entry_cells_by_dir[DIR_SOUTH] = 3;

        // exit_cells_by_dir[DIR_WEST] = 3;
    }
    else if (type == SectionType::Corner_LEFT_TOP) {
        width = 1;
        height = 2;

        entries = {6};
        exits = {6};

        entry_cells_by_dir[DIR_EAST] = 6;

        exit_cells_by_dir[DIR_SOUTH] = 6;

        // entries = {3};
        // exits = {3};

        // entry_cells_by_dir[DIR_EAST] = 3;

        // exit_cells_by_dir[DIR_SOUTH] = 3;
    }
}

// 0~8번 노드 간의 최단 거리를 미리 계산 (All-Pairs Shortest Path)
// 노드가 9개뿐이라 BFS 9번 돌려도 순식간에 끝남.
void SectionTemplate::precompute_all() {
    // 매트릭스 크기 및 초기화 (9x9)
    internal_dist_matrix.assign(9, vector<int>(9, std::numeric_limits<int>::max())); // INF로 초기화

    // 모든 시작점(Start Node)에 대해 BFS 수행
    for (int start = 0; start < 9; ++start) {
        
        // --- BFS 초기화 ---
        std::queue<int> q;
        q.push(start);
        
        // 거리와 부모 노드 기록 (경로 복원용)
        vector<int> dist(9, std::numeric_limits<int>::max());
        vector<int> parent(9, -1);
        
        dist[start] = 0;
        parent[start] = start; // 시작점 표시

        // --- BFS 탐색 ---
        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            for (int next : adj[curr]) {
                if (dist[next] == std::numeric_limits<int>::max()) { // 방문 안 했으면
                    dist[next] = dist[curr] + 1;
                    parent[next] = curr;
                    q.push(next);
                }
            }
        }

        // --- 결과 저장 (Derived Data 채우기) ---
        for (int goal = 0; goal < 9; ++goal) {
            
            // 1. Distance Matrix 채우기
            internal_dist_matrix[start][goal] = dist[goal];

            // 2. Path Table 채우기 (도달 가능한 경우만)
            if (dist[goal] != std::numeric_limits<int>::max()) {
                vector<int> path;
                int curr = goal;
                
                // 역추적 (Backtracking)
                while (curr != start) {
                    path.push_back(curr);
                    curr = parent[curr];
                }
                path.push_back(start);
                
                // 뒤집어서 정순서로 저장 (Start -> ... -> Goal)
                reverse(path.begin(), path.end());
                path_table[start][goal] = path;
            }
        }
    }
}

// ================= Section 구현 =================

Section::Section(int _id, const SectionTemplate* _info, int _gx, int _gy, int _ax, int _ay)
    : id(_id), info(_info), grid_x(_gx), grid_y(_gy), anchor_x(_ax), anchor_y(_ay) {}

// [Capacity] 해당 시간 구간 동안 섹션 전체 인원이 꽉 찼는지 확인
bool Section::check_capacity_available(int entry_time, int duration) {
    for (int t = entry_time; t < entry_time + duration; ++t) {
        // 맵에서 꺼내서 비트 수(사람 수)만 세면 끝!
        if (get_agent_count(t) >= info->capacity) {
            return false;
        }
    }
    return true;
}

// 시간 t에 secthon안의 agent 수
int Section::get_agent_count(int t) const {
    auto it = bottleneck_timeline.find(t);
    if (it == bottleneck_timeline.end()) return 0;

    return __builtin_popcount(it->second);
}

// 시간 time에 section의 index cell_idx를 사용할 수 있는가?
bool Section::check_cell_available(int time, int cell_idx) {
    // 1. 해당 시간의 타임라인 조회
    auto it = bottleneck_timeline.find(time);
    
    // 2. 해당 시간에 아무런 예약이 없으면 -> 당연히 해당 셀도 비어있음 (OK)
    if (it == bottleneck_timeline.end()) return true; 

    // 3. Bitmask Check: 해당 인덱스의 비트가 1인지 확인
    // (MapValue >> cell_idx) & 1
    // 1이면 누군가 사용 중(Collision), 0이면 비어있음
    if ((it->second >> cell_idx) & 1) {
        return false; // 이미 사용 중
    }
    
    return true; // 비어있음
}

vector<pair<int, int>> Section::get_internal_path(int start_time, int entry_idx, int exit_idx,
                                                  const vector<int>& wait_nodes) {
    vector<pair<int, int>> full_path;
    
    // [수정된 부분] 
    // 1. Template에 미리 계산해둔 path_table에 직접 접근합니다.
    // 복사를 방지하기 위해 const reference(&)를 사용합니다.

    const vector<int>& static_path = info->path_table[entry_idx][exit_idx];

    // (예외 처리: 만약 경로가 없다면 빈 벡터가 반환되므로 아래 루프가 안 돌고 안전함)
    if (static_path.empty() && entry_idx != exit_idx) {
        // 경로가 없는데 입/출구가 다르다면 오류 상황 (로깅 필요)
        return full_path; 
    }
    
    // 2. 대기 리스트를 카운팅 맵으로 변환
    // wait_nodes = {4, 4, 6} -> counts[4]=2, counts[6]=1
    std::unordered_map<int, int> wait_counts;
    for(int node : wait_nodes) {
        wait_counts[node]++;
    }
    
    int current_time = start_time;

    

    // 3. 경로 전개 (Traverse)
    for (int cell_idx : static_path) {
        
        // A. 이동: 해당 셀에 도착
        //std::cout << "full_path_update"  << cell_idx << std::endl;
        full_path.push_back({current_time, cell_idx});
        
        // B. 대기: 여기서 기다려야 하는지 확인
        if (wait_counts.find(cell_idx) != wait_counts.end()) {
            int duration = wait_counts[cell_idx];
            
            // 대기 횟수만큼 제자리 걸음 (시간만 증가, 위치는 그대로)
            for(int k=0; k<duration; ++k) {
                current_time++;
                //std::cout << "full_path_update"  << cell_idx << std::endl;
                full_path.push_back({current_time, cell_idx});
            }
        }
        
        // C. 다음 칸으로 가기 위해 시간 증가
        current_time++;
    }
    //std::cout << "full_path_size" << full_path.size() << std::endl;
    return full_path;
}

// 2. 예약 추가 (여기서만 테이블 확인)
bool Section::add_reservation(int entry_time, int entry_idx, int exit_idx, 
                              const vector<int>& wait_nodes) {
    
    vector<pair<int, int>> path = get_internal_path(entry_time, entry_idx, exit_idx, wait_nodes);
    
    // [Check Phase]
    for(auto& step : path) {
        int t = step.first;
        int idx = step.second;
        
        // 1. Capacity Check
        // 주의: get_agent_count(t)는 현재 예약된 인원 수입니다.
        // 내가 들어가면 +1이 되므로, (현재 인원 >= capacity)이면 못 들어가는 게 맞습니다.
        if (get_agent_count(t) >= info->capacity) {
            // cout << "Fail: Capacity Full at time " << t << endl; // 디버깅용
            return false;
        }

        // 2. Cell Conflict Check
        auto it = bottleneck_timeline.find(t);
        if (it != bottleneck_timeline.end()) {
            if (it->second & (1 << idx)) {
                // cout << "Fail: Cell " << idx << " occupied at time " << t << endl; // 디버깅용
                return false; 
            }
        }
    }

    // [Commit Phase]
    for(auto& step : path) {
        bottleneck_timeline[step.first] |= (1 << step.second);
    }
    
    return true;
}

void Section::remove_reservation(int entry_time, int entry_idx, int exit_idx, 
                                 const vector<int>& wait_nodes) {
    
    // 1. 예약할 때와 동일한 경로(시간-위치 쌍)를 복원
    // (determinisitic한 경로이므로 add할 때와 똑같은 path가 나옴)
    vector<pair<int, int>> path = get_internal_path(entry_time, entry_idx, exit_idx, wait_nodes);
    
    // 2. 비트마스크 해제 (Bitwise OFF)
    for(auto& step : path) {
        int t = step.first;
        int idx = step.second;
        
        auto it = bottleneck_timeline.find(t);
        if (it != bottleneck_timeline.end()) {
            
            // [핵심 로직] 해당 인덱스의 비트만 0으로 끕니다.
            // ~(1 << idx) : idx번째 비트만 0이고 나머지는 1인 마스크
            // AND 연산(&)을 하면 idx번째 비트만 강제로 0이 됨
            it->second &= ~(1 << idx);
            
            // [메모리 최적화]
            // 만약 비트가 모두 0이면 (즉, 이 시간에 아무도 없으면)
            // 맵에서 해당 시간(Key) 자체를 삭제합니다.
            if (it->second == 0) {
                bottleneck_timeline.erase(it);
            }
        }
    }
}


/*
int main() {
    cout << "=== Section Template Precomputation Test ===\n" << endl;

    SectionTemplate mySection(SectionType::Induct_C_BOTTOM, 9);

    // 2. Distance Matrix 출력
    cout << "[Internal Distance Matrix (9x9)]" << endl;
    cout << "From \\ To |";
    for(int i=0; i<9; ++i) cout << setw(3) << i << " ";
    cout << "\n----------+" << string(36, '-') << endl;

    for (int r = 0; r < 9; ++r) {
        cout << "    " << r << "     |";
        for (int c = 0; c < 9; ++c) {
            int dist = mySection.internal_dist_matrix[r][c];
            if (dist == numeric_limits<int>::max()) {
                cout << setw(3) << "-" << " "; // 갈 수 없으면 -
            } else {
                cout << setw(3) << dist << " ";
            }
        }
        cout << endl;
    }
    cout << endl;

    // 3. Path Generation Test (경로 검증)
    cout << "[Path Verification Examples]" << endl;

    // Case A: 0번에서 8번으로 가는 경로 (외곽 순환)
    // 예상: 0 -> 1 -> 2 -> 5 -> 8 (거리 4)
    int start = 0, goal = 5;
    cout << "Path " << start << " -> " << goal << ": ";
    if (mySection.path_table[start][goal].empty()) {
        cout << "No Path!" << endl;
    } else {
        for (int node : mySection.path_table[start][goal]) cout << node << " ";
        cout << "(Dist: " << mySection.internal_dist_matrix[start][goal] << ")" << endl;
    }

    // Case B: 1번에서 7번으로 가는 경로 (지름길 테스트)
    // 예상 1: 1 -> 2 -> 5 -> 8 -> 7 (거리 4, 외곽)
    // 예상 2: 1 -> 4 -> 7 (거리 2, 지름길) -> BFS니까 이걸 선택해야 함!
    start = 1; goal = 4;
    cout << "Path " << start << " -> " << goal << ": ";
    if (mySection.path_table[start][goal].empty()) {
        cout << "No Path!" << endl;
    } else {
        for (int node : mySection.path_table[start][goal]) cout << node << " ";
        cout << "(Dist: " << mySection.internal_dist_matrix[start][goal] << ")" << endl;
    }

    // Case C: 3번에서 4번 (역주행 불가 테스트)
    // 3번은 0번으로만 가는데, 4번으로 갈 수 있나? (아마 멀리 돌아가야 하거나 못 감)
    // 3->0->1->4 (거리 3)
    start = 3; goal = 4;
    cout << "Path " << start << " -> " << goal << ": ";
    if (mySection.path_table[start][goal].empty()) {
        cout << "No Path!" << endl;
    } else {
        for (int node : mySection.path_table[start][goal]) cout << node << " ";
        cout << "(Dist: " << mySection.internal_dist_matrix[start][goal] << ")" << endl;
    }

    return 0;
}

*/

/*
int main() {
    // 1. 템플릿 생성 (기존 코드)
    SectionTemplate tmpl(SectionType::Induct_C_BOTTOM, 3); // 정원 3명으로 제한 테스트
    
    // 2. 섹션 인스턴스 생성
    Section section101(101, &tmpl, 0, 0, 0, 0);

    cout << "\n=== Section Reservation Test ===\n";

    // Case 1: Agent A 예약 (시간 0에 0번 진입 -> 2번 진출)
    // 경로: 0 -> 1 -> 2 (3틱 소요: T=0, 1, 2)
    bool resA = section101.add_reservation(0, 0, 2, {});
    cout << "Agent A (0->2, T=0): " << (resA ? "Success" : "Fail") << endl;

    // Case 2: Agent B 예약 (시간 0에 0번 진입 -> 충돌 예상!)
    // A가 T=0에 0번 셀에 있으므로 충돌해야 함
    bool resB = section101.add_reservation(0, 0, 2, {});
    cout << "Agent B (0->2, T=0): " << (resB ? "Success" : "Fail (Expected)") << endl;

    // Case 3: Agent C 예약 (시간 1에 0번 진입 -> 뒤따라가기)
    // A는 T=1에 1번 셀에 있음. 0번 셀은 비어있음. 성공해야 함.
    bool resC = section101.add_reservation(1, 0, 2, {});
    cout << "Agent C (0->2, T=1): " << (resC ? "Success" : "Fail") << endl;

    // Case 4: Capacity 테스트 (정원 3명)
    // 현재 섹션에는 A와 C가 있음 (특정 시간에). 
    // Agent D, E를 추가해서 정원 초과가 뜨는지 확인 가능.
    
    // Case 5: 예약 취소 테스트
    cout << "Removing Agent A..." << endl;
    section101.remove_reservation(0, 0, 2, {});
    
    // A가 사라졌으니 이제 시간 0에 예약 가능해야 함
    bool resRetry = section101.add_reservation(0, 0, 2, {});
    cout << "Agent Retry (0->2, T=0): " << (resRetry ? "Success" : "Fail") << endl;

    return 0;
}

*/