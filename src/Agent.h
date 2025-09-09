/*
 * Agent.h
 * RVO2-3D Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/**
 * \file    Agent.h
 * \brief   Contains the Agent class.
 */
#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

#include "API.h"

#include <cstddef>
#include <utility>
#include <vector>

#include "RVOSimulator.h"
#include "Vector3.h"

namespace RVO {
	/**
	 * \brief   Defines an agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \brief   Constructs an agent instance.
		 * \param   sim  The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim);

		/**
		 * \brief   Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief   Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief   Inserts an agent neighbor into the set of neighbors of this agent.
		 * \param   agent    A pointer to the agent to be inserted.
		 * \param   rangeSq  The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);

		/**
		 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
		 */
		void update();

		/**
		 * \brief   Applies directional speed limits to a velocity vector.
		 * \param   velocity  The velocity vector to limit.
		 * \return  The velocity vector with directional speed limits applied.
		 */
		Vector3 applyDirectionalSpeedLimits(const Vector3 &velocity);

		/**
		 * \brief   Computes adaptive preferred velocity for goal proximity situations.
		 * \return  The adaptive preferred velocity vector.
		 */
		Vector3 getAdaptivePrefVelocity();

		/**
		 * \brief   Applies aggressive motion correction to prevent deadlock near goals.
		 */
		void applyAggressiveMotionCorrection();

		Vector3 newVelocity_;
		Vector3 position_;
		Vector3 prefVelocity_;
		Vector3 velocity_;
		RVOSimulator *sim_;
		size_t id_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		float radius_;
		float timeHorizon_;
		float maxAcceleration_;
		float maxDeceleration_;
		
		// 方向別速度制限用メンバー変数
		float maxHorizontalSpeed_;         // 水平方向の最大速度 (m/s)
		float maxVerticalUpSpeed_;         // 上昇方向の最大速度 (m/s)
		float maxVerticalDownSpeed_;       // 下降方向の最大速度 (m/s)
		bool useDirectionalSpeedLimits_;   // 方向別制限を使用するかのフラグ
		
		// 収束改善用のインスタンス変数
		int consecutiveLowMotionSteps_;    // 低速状態の連続ステップ数（各エージェント独立）
		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		std::vector<Plane> orcaPlanes_;

		friend class KdTree;
		friend class RVOSimulator;
	};
}

#endif /* RVO_AGENT_H_ */
