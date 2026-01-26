#include "SectionState.h"
#include "common.h"
#include "Section.h"
#include <vector>


SectionState SectionState::wait(int current_idx) const
{
    std::vector<int> next_wait_list = wait_list;
    
    // 2. 대기하는 시점의 index를 기록
    next_wait_list.push_back(current_idx);

    // 3. timestep(진입 시간)은 변하지 않음. 
    //    (진입은 이미 했고, 그 이후 과정에서 대기가 발생한 것이므로)
    return SectionState(section_id, start_index, exit_index, timestep, goal_index, next_wait_list); 
}

std::ostream & operator << (std::ostream &out, const SectionState &s)
{
    out << "[Sec:" << s.section_id 
        << " In:" << s.start_index 
        << " Out:" << (s.exit_index == -1 ? "None" : std::to_string(s.exit_index))
        << " T:" << s.timestep;
        
    if (!s.wait_list.empty()) {
        out << " W:{";
        for(auto w : s.wait_list) out << w << " ";
        out << "}";
    }
    out << "]";
    return out;
}