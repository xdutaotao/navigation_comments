/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

// A*算法实现
AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

// 计算A*的potential
// 返回值：是否计算成功
// 输入：
//      costs：costmap
//      start_x, start_y, end_x, end_y：起始点和终点
//      cycles：最大循环次数
//      potential：potential值 （输出)
// 说明:
//      1. 初始化potential
//      2. 计算potential
//      3. 回溯路径
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {

    // 初始化保存index的queue队列，清空
    queue_.clear();

    // 计算起始index,加入到queue
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    // 初始化potential，填充所有未POT_HIGH=1*10……10
    std::fill(potential, potential + ns_, POT_HIGH);

    // 起始点potential为0
    potential[start_i] = 0;

    // 目标index
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    // 针对queue中的每个点
    while (queue_.size() > 0 && cycle < cycles) {

        // 取出queue中第一个点，作为当前点
        Index top = queue_[0];

        // 弹出堆中第一个元素，然后进行堆排序，因为有元素变化，堆需要重新排序
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        // 获取当前点的index
        int i = top.i;
        if (i == goal_i)
            return true;

        
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

// 
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {

    // 如果超出边界，返回
    if (next_i < 0 || next_i >= ns_)
        return;

    // 如果当前点的potential小于POT_HIGH，表示已经计算过，返回
    if (potential[next_i] < POT_HIGH)
        return;

    // 如果当前点的cost大于等于lethal_cost_，并且不是unknown，返回
    // 说明当前点不可达
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    // 计算当前点的potential
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    // 使用曼哈顿距离作为启发式函数
    float distance = abs(end_x - x) + abs(end_y - y);

    // potential[]+distance cost 作为A*的启发cost
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));

    // 对queue进行堆排序，小顶堆，默认是大顶堆
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
