#include "MapSystem.h"
#include "SectionState.h"
#include "Section.h"
#include "BasicGraph.h"
#include <iomanip>
#include <iostream>
#include <memory>


MapSystem::MapSystem(const BasicGraph& G): G(G){
    initialize_templates();
}

MapSystem::~MapSystem() {
    cleanup();
}

void MapSystem::initialize_templates() {
    templates[SectionType::Eject_CW]        = std::make_shared<SectionTemplate>(SectionType::Eject_CW, 8);
    templates[SectionType::Eject_CCW]       = std::make_shared<SectionTemplate>(SectionType::Eject_CCW, 8);
    templates[SectionType::Eject_UP]        = std::make_shared<SectionTemplate>(SectionType::Eject_UP, 8);
    templates[SectionType::Eject_DOWN]      = std::make_shared<SectionTemplate>(SectionType::Eject_DOWN, 8);
    templates[SectionType::Induct_C_BOTTOM] = std::make_shared<SectionTemplate>(SectionType::Induct_C_BOTTOM, 6);
    templates[SectionType::Induct_N_BOTTOM] = std::make_shared<SectionTemplate>(SectionType::Induct_N_BOTTOM, 6);
    templates[SectionType::Induct_C_TOP]    = std::make_shared<SectionTemplate>(SectionType::Induct_C_TOP, 6);
    templates[SectionType::Induct_N_TOP]    = std::make_shared<SectionTemplate>(SectionType::Induct_N_TOP, 6);
    templates[SectionType::Travel_LEFT_BOTTOM] = std::make_shared<SectionTemplate>(SectionType::Travel_LEFT_BOTTOM, 3);
    templates[SectionType::Travel_LEFT_TOP] = std::make_shared<SectionTemplate>(SectionType::Travel_LEFT_TOP, 3);
    templates[SectionType::Travel_RIGHT_BOTTOM]    = std::make_shared<SectionTemplate>(SectionType::Travel_RIGHT_BOTTOM, 3);
    templates[SectionType::Travel_RIGHT_TOP]    = std::make_shared<SectionTemplate>(SectionType::Travel_RIGHT_TOP, 3);
    templates[SectionType::Corner_LEFT_BOTTOM] = std::make_shared<SectionTemplate>(SectionType::Corner_LEFT_BOTTOM, 1);
    templates[SectionType::Corner_LEFT_TOP] = std::make_shared<SectionTemplate>(SectionType::Corner_LEFT_TOP, 1);
    templates[SectionType::Corner_RIGHT_BOTTOM]    = std::make_shared<SectionTemplate>(SectionType::Corner_RIGHT_BOTTOM, 1);
    templates[SectionType::Corner_RIGHT_TOP]    = std::make_shared<SectionTemplate>(SectionType::Corner_RIGHT_TOP, 1);

    cout << "[MapSystem] Templates Initialized." << endl;
}

void MapSystem::cleanup() {
    for (auto& pair : sections) {
        delete pair.second; // new로 생성한 Section 객체 해제
    }
    sections.clear();
}

Section* MapSystem::add_section(int grid_x, int grid_y, int anchor_x, int anchor_y, SectionType type) {
    
    if (sections.find({grid_x, grid_y}) != sections.end()) {
        std::cerr << "[Error] Section already exists at (" << grid_x << ", " << grid_y << ")" << std::endl;
        return nullptr;
    }

    if (templates.find(type) == templates.end()) {
        std::cerr << "[Error] Unknown Section Type!" << std::endl;
        return nullptr;
    }

    int section_id = (grid_y * 1000) + grid_x;

    Section* new_section = new Section(section_id, templates[type].get(), grid_x, grid_y, anchor_x, anchor_y);
    
    // 맵에 등록
    sections[{grid_x, grid_y}] = new_section;
    sections_by_id[section_id] = new_section;
    
    // 범위 갱신
    if (grid_x > max_grid_x) max_grid_x = grid_x;
    if (grid_y > max_grid_y) max_grid_y = grid_y;

    int w = new_section->info->width;
    int h = new_section->info->height;

    for (int dx = 0; dx < w; ++dx) {
        for (int dy = 0; dy < h; ++dy) {
            int current_x = anchor_x + dx;
            int current_y = anchor_y + dy;
            //std::cout << " x : " << current_x << " y : " << current_y << std::endl;
            // (current_x, current_y) 칸의 주인은 new_sec이다! 라고 등록
            cell_registry[{current_x, current_y}] = new_section;
        }
    }

    return new_section;
}

void MapSystem::link_all_sections() {
    cout << "[MapSystem] Linking Sections..." << endl;


    // 0:East, 1:South, 2:West, 3:North
    int dx[] = {1, 0, -1, 0};
    int dy[] = {0, -1, 0, 1};

    for (auto& item : sections) {
        Section* current = item.second;
        int cx = current->grid_x;
        int cy = current->grid_y;

        for (int dir = 0; dir < 4; dir++) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];

            // 이웃 검색

            auto it = sections.find({nx, ny});
            if (it != sections.end()) {
                Section* neighbor = it->second;
                try_connect(current, neighbor, dir);
            }
        }
    }
    cout << "[MapSystem] Linking Complete." << endl;
}

Section* MapSystem::get_section(int gx, int gy) {
    auto it = sections.find({gx, gy});
    if (it != sections.end()) {
        return it->second;
    }
    return nullptr;
}

void MapSystem::try_connect(Section* from, Section* to, int dir_from_to) {
    // 반대 방향 (to 기준 from의 방향)
    int dir_to_from = (dir_from_to + 2) % 4;

    // 템플릿의 방향 정보 참조
    // (Section -> SectionTemplate -> vector 접근)
    int exit_idx = from->info->exit_cells_by_dir[dir_from_to];
    int entry_idx = to->info->entry_cells_by_dir[dir_to_from];

    // 둘 다 연결 가능한 포트가 있다면 연결
    if (exit_idx != -1 && entry_idx != -1) {
        if (dir_from_to == 1 || dir_from_to == 3){
            if ((exit_idx % 3) == (entry_idx % 3)){
                from->neighbors[exit_idx].push_back({to, entry_idx});
            }
        }
        else{
            if (exit_idx / 3 == entry_idx / 3){
                from->neighbors[exit_idx].push_back({to, entry_idx});
            }
        }
        
        
        // (디버그 출력)
        // cout << "Link: Sec " << from->id << " -> Sec " << to->id << endl;
    }
}

void MapSystem::build_procedural_map(int grid_cols, int grid_rows) {
    SectionType type = SectionType::Eject_CW;
    const int E_WIDTH = 3;
    const int E_HEIGHT = 3;
    int logical_cols = (grid_cols-4)/E_HEIGHT + 2;
    int logical_rows = (grid_rows-2)/E_WIDTH + 2;
    cout << "[MapSystem] Generating Procedural Map" << endl;
    cout << "  - Raw Grid: " << grid_cols << " x " << grid_rows << endl;
    cout << "  - Logical:  " << logical_cols << " x " << logical_rows << endl;
    
    int current_anchor_y = 0;
    

    for (int ly = 0; ly < logical_cols; ++ly) {
        int current_anchor_x = 0;
        int max_row_height = 0;

        // [행 타입 판별]
        bool is_bottom_row = (ly == 0);
        bool is_top_row = (ly == logical_cols - 1);
        
        // 중간 행 패턴 결정 (이미지 구조에 맞춤)
        // Row 1 (바닥 바로 위): Travel_LEFT_BOTTOM + Eject_CCW ...
        // Row 2: Travel_LEFT_TOP + Eject_DOWN ...
        // 이를 위해 (ly % 2)로 분기 처리
        bool is_ccw_row = (ly % 2 != 0); // 짝수 행이라 가정 (조정 가능)

        for (int lx = 0; lx < logical_rows; ++lx) {
            SectionType type;
            int width = 3;  // 기본 너비
            int height = 3; // 기본 높이

            // -------------------------------------------------
            // 1. 왼쪽 가장자리 (Left Edge, x=0)
            // -------------------------------------------------
            if (lx == 0) {
                width = 1; // 1x3 크기 가정
                if (is_bottom_row) type = SectionType::Corner_LEFT_BOTTOM;
                else if (is_top_row) type = SectionType::Corner_LEFT_TOP;
                else {
                    // 중간 행: 교차 패턴
                    if (is_ccw_row) type = SectionType::Travel_LEFT_BOTTOM;
                    else type = SectionType::Travel_LEFT_TOP;
                }
            }
            // -------------------------------------------------
            // 2. 오른쪽 가장자리 (Right Edge, x=end)
            // -------------------------------------------------
            else if (lx == logical_rows - 1) {
                width = 1;
                if (is_bottom_row) type = SectionType::Corner_RIGHT_BOTTOM;
                else if (is_top_row) type = SectionType::Corner_RIGHT_TOP;
                else {
                    // 중간 행: 교차 패턴
                    if (is_ccw_row) type = SectionType::Travel_RIGHT_BOTTOM;
                    else type = SectionType::Travel_RIGHT_TOP;
                }
            }
            // -------------------------------------------------
            // 3. 중앙 영역 (Center Area)
            // -------------------------------------------------
            else {
                // 패턴 인덱스 (0, 1, 0, 1...) : Induct나 Eject가 2개씩 짝지어 반복됨
                // lx=1부터 시작하므로 (lx-1)을 해줌
                bool is_even_col = ((lx - 1) % 2 == 0);

                if (is_bottom_row) {
                    // [BOTTOM] Induct_N, Induct_C 반복
                    width = 3; // Eject와 열을 맞추기 위해 3으로 설정 (실제 물리폭 2라면 조정 필요)
                    if (is_even_col) type = SectionType::Induct_N_BOTTOM;
                    else type = SectionType::Induct_C_BOTTOM;
                }
                else if (is_top_row) {
                    // [TOP] Induct_N, Induct_C 반복
                    width = 3; 
                    if (is_even_col) type = SectionType::Induct_N_TOP;
                    else type = SectionType::Induct_C_TOP;
                }
                else {
                    // [MIDDLE] Eject 패턴
                    if (is_ccw_row) {
                        // 이미지: Travel_BOTTOM 줄에는 [Eject_CCW, Eject_UP]
                        if (is_even_col) type = SectionType::Eject_CCW;
                        else type = SectionType::Eject_UP;
                    } 
                    else {
                        // 이미지: Travel_TOP 줄에는 [Eject_DOWN, Eject_CW]
                        if (is_even_col) type = SectionType::Eject_DOWN;
                        else type = SectionType::Eject_CW;
                    }
                }
            }

            // -------------------------------------------------
            // 섹션 생성 및 좌표 누적
            // -------------------------------------------------
            Section* new_section = add_section(lx, ly, current_anchor_x, current_anchor_y, type);
            width = new_section->info->width;
            height = new_section->info->height;


            current_anchor_x += width;
            if (height > max_row_height) max_row_height = height;
        }

        current_anchor_y += max_row_height;
 
    }

    // 연결 수행
    link_all_sections();
}

pair<Section*, int> MapSystem::get_section_at_grid(int grid_x, int grid_y) {
    if (cell_registry.find({grid_x, grid_y}) != cell_registry.end()) {
        Section* cur_section = cell_registry[{grid_x, grid_y}];
        int grid_diff_x = grid_x - cur_section->anchor_x;
        int grid_diff_y = grid_y - cur_section->anchor_y;

        //std::cout << "grid_x : " << grid_x << " grid_ y : " << grid_y << std::endl;
        //std::cout << "std_x : " << cur_section->anchor_x << " std_ y : " << cur_section->anchor_y << std::endl;
        //std::cout << "diff_x : " << grid_diff_x << " diff_ y : " << grid_diff_y << std::endl;
        /*
        int index = (-grid_diff_y) * 3 + grid_diff_x;
        return {cell_registry[{grid_x, grid_y}], index};
        */
            
        if (grid_diff_x == 0){
            if (grid_diff_y == 0){
                return {cell_registry[{grid_x, grid_y}], 6};
            }
            else if (grid_diff_y == 1){
                return {cell_registry[{grid_x, grid_y}], 3};
            }
            else if (grid_diff_y == 2){
                return {cell_registry[{grid_x, grid_y}], 0};
            }
        }
        else if (grid_diff_x == 1){
            if (grid_diff_y == 0){
                return {cell_registry[{grid_x, grid_y}], 7};
            }
            else if (grid_diff_y == 1){
                return {cell_registry[{grid_x, grid_y}], 4};
            }
            else if (grid_diff_y == 2){
                return {cell_registry[{grid_x, grid_y}], 1};
            }
        }
        else if (grid_diff_x == 2){
            if (grid_diff_y == 0){
                return {cell_registry[{grid_x, grid_y}], 8};
            }
            else if (grid_diff_y == 1){
                return {cell_registry[{grid_x, grid_y}], 5};
            }
            else if (grid_diff_y == 2){
                return {cell_registry[{grid_x, grid_y}], 2};
            }
        }
        else{
            cout << "Something Wrong" << endl;
            return {nullptr, -1};
        }
    }
    return {nullptr, -1}; // 빈 땅(Obstacle 등)
}

string get_type_name(SectionType type) {
    switch (type) {
        case SectionType::Eject_CW:   return "E_CW";
        case SectionType::Eject_CCW:  return "E_CCW";
        case SectionType::Eject_DOWN: return "E_DOWN";
        case SectionType::Eject_UP:   return "E_UP";
        case SectionType::Induct_N_TOP: return "I_N_T";
        case SectionType::Induct_C_TOP:    return "I_N_T";
        case SectionType::Induct_N_BOTTOM:    return "I_N_B";
        case SectionType::Induct_C_BOTTOM:    return "I_C_B";
        case SectionType::Travel_LEFT_BOTTOM: return "T_L_B";
        case SectionType::Travel_LEFT_TOP:    return "T_L_T";
        case SectionType::Travel_RIGHT_BOTTOM:    return "T_R_B";
        case SectionType::Travel_RIGHT_TOP:    return "T_R_T";
        case SectionType::Corner_LEFT_BOTTOM: return "C_L_B";
        case SectionType::Corner_LEFT_TOP:    return "C_L_T";
        case SectionType::Corner_RIGHT_BOTTOM:    return "C_R_B";
        case SectionType::Corner_RIGHT_TOP:    return "C_R_T";
        

        // ... 필요한 다른 타입들도 추가 가능
        default: return "Other";
    }
}

int MapSystem::get_distance(int section_id, int start_idx, int goal_idx, const vector<int>& wait_nodes){

    const auto& static_path = sections_by_id[section_id]->info->path_table[start_idx][goal_idx];

    if (static_path.empty()) {
        if (start_idx == goal_idx) return wait_nodes.size(); // 제자리 대기만 있는 경우
        return -1; // 길이 없는 경우 (오류 또는 무한대 처리)
    }

    int base_distance = static_path.size() - 1;

    int total_distance = base_distance + wait_nodes.size();

    return total_distance;
}

std::vector<int> index_x = {0, 1, 2, 0, 1, 2, 0, 1, 2};
std::vector<int> index_y = {2, 2, 2, 1, 1, 1, 0, 0, 0};

double MapSystem::compute_h_value(int current_section_id, int current_index, int current_goal_id, const vector<pair<SectionState, int>>& goal_sections) 
{
    double h_val = 0.0;

    // 1. 현재 위치 -> 바로 다음 목표(Current Goal)까지의 거리 계산
    const SectionState& current_goal = goal_sections[current_goal_id].first;

    int curr_x = sections_by_id[current_section_id]->anchor_x + index_x[current_index];
    int curr_y = sections_by_id[current_section_id]->anchor_y + index_y[current_index];
    int goal_x = sections_by_id[current_goal.section_id]->anchor_x + index_x[current_goal.goal_index];
    int goal_y = sections_by_id[current_goal.section_id]->anchor_y + index_y[current_goal.goal_index];

    int curr_cell_id = curr_x * G.cols + curr_y;
    int goal_cell_id = goal_x * G.cols + goal_y;

    h_val += G.heuristics.at(goal_cell_id)[curr_cell_id];
    current_goal_id++;
    // if (current_section_id == current_goal.section_id) {
    //     // [케이스 A] 같은 섹션 안에 목표가 있다면? -> 미리 만들어둔 O(1) 거리 함수로 정확한 거리를 구함
    //     int dist = get_distance(current_section_id, current_index, current_goal.goal_index, {});
    //     h_val += (dist != -1) ? dist : 0;
    // } 
    // else {
    //     // [케이스 B] 다른 섹션에 있다면? -> 장애물이 없는 텅 빈 맵 기준 맨해튼 거리(Manhattan Distance) 사용
    //     // (cols는 MapSystem이 들고 있는 맵의 가로 크기라고 가정합니다)
    //     int curr_x = sections_by_id[current_section_id]->anchor_x + index_x[current_index];
    //     int curr_y = sections_by_id[current_section_id]->anchor_y + index_y[current_index];
    //     int goal_x = sections_by_id[current_goal.section_id]->anchor_x + index_x[current_goal.goal_index];
    //     int goal_y = sections_by_id[current_goal.section_id]->anchor_y + index_y[current_goal.goal_index];

    //     h_val += std::abs(curr_x - goal_x) + std::abs(curr_y - goal_y);
    // }

    // ✨ 2. 다중 목적지 최적화: 아직 방문 안 한 '남은 경유지들' 사이의 거리도 모두 더해줌!
    // 예: 현재 -> 목표1 계산 끝. 이제 (목표1 -> 목표2) + (목표2 -> 목표3) 거리를 더함
    while (current_goal_id < (int) goal_sections.size()) 
    {
        const SectionState& g1 = goal_sections[current_goal_id - 1].first;
        const SectionState& g2 = goal_sections[current_goal_id].first;

        int g1_x = sections_by_id[g1.section_id]->anchor_x + index_x[g1.goal_index];
        int g1_y = sections_by_id[g1.section_id]->anchor_y + index_y[g1.goal_index];
        int g2_x = sections_by_id[g2.section_id]->anchor_x + index_x[g2.goal_index];
        int g2_y = sections_by_id[g2.section_id]->anchor_y + index_y[g2.goal_index];

        int g1_cell_id = g1_x * G.cols + g1_y;
        int g2_cell_id = g2_x * G.cols + g2_y;

        // 경유지들 사이의 맨해튼 거리 누적
        h_val += G.heuristics.at(g2_cell_id)[g1_cell_id];
        current_goal_id++;
    }

    return h_val;
}

/* 
int main() {
    // 1. 시스템 초기화
    MapSystem mapSys;

    // 2. 템플릿 등록 (테스트를 위해 실제 템플릿들이 등록되어 있어야 합니다)
    // 예: mapSys.register_template(SectionType::Eject_CW, 3, 3, ...);
    cout << "[Step 1] Templates initialized." << endl;

    // 3. 절차적 맵 생성 (37 x 77 그리드 기준)
    cout << "[Step 2] Building procedural map (37x77)..." << endl;
    mapSys.build_procedural_map(37, 77);

    // 4. 결과 검증 출력
    cout << "\n[Step 3] Verification - Section List (Partial)" << endl;
    cout << "---------------------------------------------------------------------------------------------------" << endl;
    cout << std::setw(6) << "ID" << std::setw(12) << "Logical(X,Y)" << std::setw(13) << "Anchor(X,Y)" 
         << std::setw(12) << "Type" << "  |  " << "Neighbors (Port:TargetID)" << endl;
    cout << "---------------------------------------------------------------------------------------------------" << endl;

    int count = 0;
    auto& all_sections = mapSys.get_sections(); // Getter 사용

    for (auto const& [pos, sec] : all_sections) {
        // 너무 많으면 처음 15개와 마지막 5개만 출력
        if (count < 10 || count > (int)all_sections.size() - 350) {
            
            // [수정 1] ID, 좌표 출력
            cout << std::setw(6) << sec->id 
                 << std::setw(5) << "(" << pos.first << "," << pos.second << ")"
                 << std::setw(8) << "(" << sec->anchor_x << "," << sec->anchor_y << ")";

            // [수정 2] sec->info->type 접근
            if (sec->info) {
                cout << std::setw(12) << get_type_name(sec->info->type);
            } else {
                cout << std::setw(12) << "No Info";
            }

            cout << "  |  ";

            // [수정 3] Neighbors 출력 (PortConnection 구조체 반영)
            if (sec->neighbors.empty()) {
                cout << "None";
            } else {
                for (auto const& [exit_idx, conn_list] : sec->neighbors) {
                    // conn_list는 이제 'PortConnection' 하나가 아니라 'vector<PortConnection>' 입니다.
                    // 그래서 리스트 안을 한 번 더 돌아야 합니다.
                    for (auto const& conn : conn_list) {
                        if (conn.target_sec != nullptr) {
                            cout << "[" << exit_idx << ":" << conn.target_sec->id << "] ";
                        }
                    }
                }
            }
            cout << endl;

        } else if (count == 15) {
            cout << "  ... (middle sections omitted) ..." << endl;
        }
        count++;
    }
    
    cout << "---------------------------------------------------------------------------------------------------" << endl;
    cout << "Total Sections Generated: " << all_sections.size() << endl;

    return 0;
}
 */


