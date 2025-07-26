# distutils: language = c++
from libcpp.vector cimport vector
from libcpp cimport bool


cdef extern from "Vector3.h" namespace "RVO":
    cdef cppclass Vector3:
        Vector3() except +
        Vector3(float x, float y, float y) except +
        float x() const
        float y() const
        float z() const


cdef extern from "RVOSimulator.h" namespace "RVO":
    cdef const size_t RVO_ERROR


cdef extern from "RVOSimulator.h" namespace "RVO":
    cdef cppclass Line:
        Vector3 point
        Vector3 direction


cdef extern from "RVOSimulator.h" namespace "RVO":
    cdef cppclass RVOSimulator:
        RVOSimulator()
        RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors,
                     float timeHorizon, float radius,
                     float maxSpeed, const Vector3 & velocity)
        size_t addAgent(const Vector3 & position)
        size_t addAgent(const Vector3 & position, float neighborDist,
                        size_t maxNeighbors, float timeHorizon,
                        float radius, float maxSpeed,
                        const Vector3 & velocity)
        void doStep() nogil
        size_t getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
        size_t getAgentMaxNeighbors(size_t agentNo) const
        float getAgentMaxSpeed(size_t agentNo) const
        float getAgentNeighborDist(size_t agentNo) const
        size_t getAgentNumAgentNeighbors(size_t agentNo) const
        const Vector3 & getAgentPosition(size_t agentNo) const
        const Vector3 & getAgentPrefVelocity(size_t agentNo) const
        float getAgentRadius(size_t agentNo) const
        float getAgentTimeHorizon(size_t agentNo) const
        const Vector3 & getAgentVelocity(size_t agentNo) const
        float getGlobalTime() const
        size_t getNumAgents() const
        float getTimeStep() const

        bool queryVisibility(const Vector3 & point1, const Vector3 & point2,
                             float radius) nogil const
        void setAgentDefaults(float neighborDist, size_t maxNeighbors,
                              float timeHorizon,
                              float radius, float maxSpeed,
                              const Vector3 & velocity)
        void setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
        void setAgentMaxSpeed(size_t agentNo, float maxSpeed)
        void setAgentNeighborDist(size_t agentNo, float neighborDist)
        void setAgentPosition(size_t agentNo, const Vector3 & position)
        void setAgentPrefVelocity(size_t agentNo, const Vector3 & prefVelocity)
        void setAgentRadius(size_t agentNo, float radius)
        void setAgentTimeHorizon(size_t agentNo, float timeHorizon)

        void setAgentVelocity(size_t agentNo, const Vector3 & velocity)
        void setTimeStep(float timeStep)
        
        # 加速度制限機能
        float getAgentMaxAcceleration(size_t agentNo) const
        float getAgentMaxDeceleration(size_t agentNo) const
        void setAgentMaxAcceleration(size_t agentNo, float maxAcceleration)
        void setAgentMaxDeceleration(size_t agentNo, float maxDeceleration)

        # 方向別速度制限機能
        float getAgentHorizontalSpeed(size_t agentNo) const
        float getAgentVerticalUpSpeed(size_t agentNo) const
        float getAgentVerticalDownSpeed(size_t agentNo) const
        bool getAgentUseDirectionalSpeedLimits(size_t agentNo) const
        void setAgentHorizontalSpeed(size_t agentNo, float maxHorizontalSpeed)
        void setAgentVerticalUpSpeed(size_t agentNo, float maxVerticalUpSpeed)
        void setAgentVerticalDownSpeed(size_t agentNo, float maxVerticalDownSpeed)
        void setAgentDirectionalSpeeds(size_t agentNo, float maxHorizontalSpeed, float maxVerticalUpSpeed, float maxVerticalDownSpeed)
        void setAgentUseDirectionalSpeedLimits(size_t agentNo, bool use)


cdef class PyRVOSimulator:
    cdef RVOSimulator *thisptr

    def __cinit__(self, float timeStep, float neighborDist, size_t maxNeighbors,
                  float timeHorizon, float radius,
                  float maxSpeed, tuple velocity=(0, 0, 0)):
        cdef Vector3 c_velocity = Vector3(velocity[0], velocity[1], velocity[2])

        self.thisptr = new RVOSimulator(timeStep, neighborDist, maxNeighbors,
                                        timeHorizon, radius,
                                        maxSpeed, c_velocity)

    def addAgent(self, tuple pos, neighborDist=None,
                 maxNeighbors=None, timeHorizon=None,
                 radius=None, maxSpeed=None,
                 velocity=None):
        cdef Vector3 c_pos = Vector3(pos[0], pos[1], pos[2])
        cdef Vector3 c_velocity

        if neighborDist is not None and velocity is None:
            raise ValueError("Either pass only 'pos', or pass all parameters.")

        if neighborDist is None:
            agent_nr = self.thisptr.addAgent(c_pos)
        else:
            c_velocity = Vector3(velocity[0], velocity[1], velocity[2])
            agent_nr = self.thisptr.addAgent(c_pos, neighborDist,
                                             maxNeighbors, timeHorizon,
                                             radius, maxSpeed,
                                             c_velocity)

        if agent_nr == RVO_ERROR:
            raise RuntimeError('Error adding agent to RVO simulation')

        return agent_nr

    def doStep(self):
        with nogil:
            self.thisptr.doStep()

    def getAgentAgentNeighbor(self, size_t agent_no, size_t neighbor_no):
        return self.thisptr.getAgentAgentNeighbor(agent_no, neighbor_no)
    def getAgentMaxNeighbors(self, size_t agent_no):
        return self.thisptr.getAgentMaxNeighbors(agent_no)
    def getAgentMaxSpeed(self, size_t agent_no):
        return self.thisptr.getAgentMaxSpeed(agent_no)
    def getAgentNeighborDist(self, size_t agent_no):
        return self.thisptr.getAgentNeighborDist(agent_no)
    def getAgentNumAgentNeighbors(self, size_t agent_no):
        return self.thisptr.getAgentNumAgentNeighbors(agent_no)
    def getAgentPosition(self, size_t agent_no):
        cdef Vector3 pos = self.thisptr.getAgentPosition(agent_no)
        return pos.x(), pos.y(), pos.z()
    def getAgentPrefVelocity(self, size_t agent_no):
        cdef Vector3 velocity = self.thisptr.getAgentPrefVelocity(agent_no)
        return velocity.x(), velocity.y(), velocity.z()
    def getAgentRadius(self, size_t agent_no):
        return self.thisptr.getAgentRadius(agent_no)
    def getAgentTimeHorizon(self, size_t agent_no):
        return self.thisptr.getAgentTimeHorizon(agent_no)
    def getAgentVelocity(self, size_t agent_no):
        cdef Vector3 velocity = self.thisptr.getAgentVelocity(agent_no)
        return velocity.x(), velocity.y(), velocity.z()
    def getGlobalTime(self):
        return self.thisptr.getGlobalTime()
    def getNumAgents(self):
        return self.thisptr.getNumAgents()
    def getTimeStep(self):
        return self.thisptr.getTimeStep()

    def setAgentDefaults(self, float neighbor_dist, size_t max_neighbors, float time_horizon,
                         float radius, float max_speed,
                         tuple velocity=(0, 0)):
        cdef Vector3 c_velocity = Vector3(velocity[0], velocity[1], velocity[2])
        self.thisptr.setAgentDefaults(neighbor_dist, max_neighbors, time_horizon,
                                      radius, max_speed, c_velocity)

    def setAgentMaxNeighbors(self, size_t agent_no, size_t max_neighbors):
        self.thisptr.setAgentMaxNeighbors(agent_no, max_neighbors)
    def setAgentMaxSpeed(self, size_t agent_no, float max_speed):
        self.thisptr.setAgentMaxSpeed(agent_no, max_speed)
    def setAgentNeighborDist(self, size_t agent_no, float neighbor_dist):
        self.thisptr.setAgentNeighborDist(agent_no, neighbor_dist)
    def setAgentNeighborDist(self, size_t agent_no, float neighbor_dist):
        self.thisptr.setAgentNeighborDist(agent_no, neighbor_dist)
    def setAgentPosition(self, size_t agent_no, tuple position):
        cdef Vector3 c_pos = Vector3(position[0], position[1], position[2])
        self.thisptr.setAgentPosition(agent_no, c_pos)
    def setAgentPrefVelocity(self, size_t agent_no, tuple velocity):
        cdef Vector3 c_velocity = Vector3(velocity[0], velocity[1], velocity[2])
        self.thisptr.setAgentPrefVelocity(agent_no, c_velocity)
    def setAgentRadius(self, size_t agent_no, float radius):
        self.thisptr.setAgentRadius(agent_no, radius)
    def setAgentTimeHorizon(self, size_t agent_no, float time_horizon):
        self.thisptr.setAgentTimeHorizon(agent_no, time_horizon)
    def setAgentVelocity(self, size_t agent_no, tuple velocity):
        cdef Vector3 c_velocity = Vector3(velocity[0], velocity[1], velocity[2])
        self.thisptr.setAgentVelocity(agent_no, c_velocity)
    def setTimeStep(self, float time_step):
        self.thisptr.setTimeStep(time_step)
    
    # 加速度制限機能のメソッド
    def getAgentMaxAcceleration(self, size_t agent_no):
        """エージェントの最大加速度を取得"""
        return self.thisptr.getAgentMaxAcceleration(agent_no)
    
    def getAgentMaxDeceleration(self, size_t agent_no):
        """エージェントの最大減速度を取得"""
        return self.thisptr.getAgentMaxDeceleration(agent_no)
    
    def setAgentMaxAcceleration(self, size_t agent_no, float max_acceleration):
        """エージェントの最大加速度を設定"""
        self.thisptr.setAgentMaxAcceleration(agent_no, max_acceleration)
    
    def setAgentMaxDeceleration(self, size_t agent_no, float max_deceleration):
        """エージェントの最大減速度を設定"""
        self.thisptr.setAgentMaxDeceleration(agent_no, max_deceleration)
    
    def setAgentAccelerationLimits(self, size_t agent_no, float max_acceleration, float max_deceleration):
        """エージェントの加速度・減速度制限を同時設定（便利メソッド）"""
        self.setAgentMaxAcceleration(agent_no, max_acceleration)
        self.setAgentMaxDeceleration(agent_no, max_deceleration)

    # 方向別速度制限機能のメソッド
    def getAgentHorizontalSpeed(self, size_t agent_no):
        """エージェントの水平方向最大速度を取得"""
        return self.thisptr.getAgentHorizontalSpeed(agent_no)
    
    def getAgentVerticalUpSpeed(self, size_t agent_no):
        """エージェントの上昇方向最大速度を取得"""
        return self.thisptr.getAgentVerticalUpSpeed(agent_no)
    
    def getAgentVerticalDownSpeed(self, size_t agent_no):
        """エージェントの下降方向最大速度を取得"""
        return self.thisptr.getAgentVerticalDownSpeed(agent_no)
    
    def getAgentUseDirectionalSpeedLimits(self, size_t agent_no):
        """エージェントが方向別速度制限を使用しているかを取得"""
        return self.thisptr.getAgentUseDirectionalSpeedLimits(agent_no)
    
    def setAgentHorizontalSpeed(self, size_t agent_no, float max_horizontal_speed):
        """エージェントの水平方向最大速度を設定"""
        self.thisptr.setAgentHorizontalSpeed(agent_no, max_horizontal_speed)
    
    def setAgentVerticalUpSpeed(self, size_t agent_no, float max_vertical_up_speed):
        """エージェントの上昇方向最大速度を設定"""
        self.thisptr.setAgentVerticalUpSpeed(agent_no, max_vertical_up_speed)
    
    def setAgentVerticalDownSpeed(self, size_t agent_no, float max_vertical_down_speed):
        """エージェントの下降方向最大速度を設定"""
        self.thisptr.setAgentVerticalDownSpeed(agent_no, max_vertical_down_speed)
    
    def setAgentDirectionalSpeeds(self, size_t agent_no, float max_horizontal_speed, float max_vertical_up_speed, float max_vertical_down_speed):
        """エージェントの方向別最大速度を一括設定（便利メソッド）"""
        self.thisptr.setAgentDirectionalSpeeds(agent_no, max_horizontal_speed, max_vertical_up_speed, max_vertical_down_speed)
    
    def setAgentUseDirectionalSpeedLimits(self, size_t agent_no, bool use):
        """エージェントの方向別速度制限使用フラグを設定"""
        self.thisptr.setAgentUseDirectionalSpeedLimits(agent_no, use)
    
    def enableDirectionalSpeedLimits(self, size_t agent_no, float horizontal_speed=4.0, float vertical_up_speed=3.2, float vertical_down_speed=2.4):
        """方向別速度制限を有効にし、指定した速度を設定（便利メソッド）
        
        Args:
            agent_no: エージェント番号
            horizontal_speed: 水平方向最大速度 (デフォルト: 4.0 m/s)
            vertical_up_speed: 上昇方向最大速度 (デフォルト: 3.2 m/s)
            vertical_down_speed: 下降方向最大速度 (デフォルト: 2.4 m/s)
        """
        self.setAgentDirectionalSpeeds(agent_no, horizontal_speed, vertical_up_speed, vertical_down_speed)
        self.setAgentUseDirectionalSpeedLimits(agent_no, True)
