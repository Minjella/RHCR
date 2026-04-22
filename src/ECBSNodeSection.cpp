#include "ECBSNodeSection.h"

ECBSNodeSection::ECBSNodeSection(ECBSNodeSection* parent)
    : in_openlist(false),
      conflict(-1, -1, -1, -1, -1, ConflictType::TILE_VERTEX),
      parent(parent),
      g_val(parent ? parent->g_val : 0),
      h_val(0),
      f_val(parent ? parent->f_val : 0),
      min_f_val(parent ? parent->min_f_val : 0),
      depth(parent ? parent->depth + 1 : 0),
      num_of_collisions(0),
      time_expanded(0),
      time_generated(0),
      window(parent ? parent->window : 0)
{
}

void ECBSNodeSection::clear()
{
    // Free heavy containers — parent chain을 통해 path가 참조되므로
    // 여기서 paths는 지우지 않는다. conflict list만 정리.
    conflicts.clear();
}
