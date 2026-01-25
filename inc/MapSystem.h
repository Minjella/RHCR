#include <vector>
#include <map>
#include <memory>
#include <utility>
#include "Section.h"


class MapSystem {
    private:
        // 1. 템플릿 저장소
        std::map<SectionType, std::shared_ptr<SectionTemplate>> templates;

        // 2. 실제 맵 데이터 (Key: 좌표, Value: 섹션 포인터)
        std::map<pair<int, int>, Section*> sections;
        std::map<int, Section*> sections_by_id;
        std::map<std::pair<int, int>, Section*> cell_registry;


        // 맵 범위 정보
        int max_grid_x = 0;
        int max_grid_y = 0;

    public:
        MapSystem();
        ~MapSystem();

        // 초기화 및 해제
        void initialize_templates();
        void cleanup();

        // 맵 구축 API
        Section* add_section(int grid_x, int grid_y, int anchor_x, int anchor_y, SectionType type);
        void build_procedural_map(int logical_cols, int logical_rows);
        void link_all_sections();
        
        // 유틸리티
        Section* get_section(int gx, int gy);
        const std::map<std::pair<int, int>, Section*>& get_sections() const { return sections; }
        pair<Section*, int> get_section_at_grid(int grid_x, int grid_y);

    private:
        // 내부 로직: 두 섹션 연결 시도
        void try_connect(Section* from, Section* to, int dir_from_to);
};


