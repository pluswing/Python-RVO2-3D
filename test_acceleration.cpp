/*
 * test_acceleration.cpp
 * 加速度制限機能のテストコード
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "RVO.h"

using namespace RVO;

// テスト結果の統計
struct TestStats {
    int totalTests = 0;
    int passedTests = 0;
    
    void recordTest(bool passed, const std::string& testName) {
        totalTests++;
        if (passed) {
            passedTests++;
            std::cout << "[PASS] " << testName << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << std::endl;
        }
    }
    
    void printSummary() {
        std::cout << "\n=== テスト結果 ===" << std::endl;
        std::cout << "総テスト数: " << totalTests << std::endl;
        std::cout << "成功: " << passedTests << std::endl;
        std::cout << "失敗: " << (totalTests - passedTests) << std::endl;
        std::cout << "成功率: " << (100.0f * passedTests / totalTests) << "%" << std::endl;
    }
};

// ベクトルの近似比較
bool isVectorNear(const Vector3& a, const Vector3& b, float tolerance = 0.001f) {
    return abs(a - b) < tolerance;
}

// 浮動小数点の近似比較
bool isFloatNear(float a, float b, float tolerance = 0.001f) {
    return std::abs(a - b) < tolerance;
}

// テスト1: API動作確認
void testAPI(TestStats& stats) {
    std::cout << "\n=== API動作テスト ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.1f);
    
    // デフォルト設定を行う（addAgentの前に必要）
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    // エージェント追加
    size_t agentId = sim->addAgent(Vector3(0, 0, 0));
    
    // デフォルト値確認
    float defaultAccel = sim->getAgentMaxAcceleration(agentId);
    float defaultDecel = sim->getAgentMaxDeceleration(agentId);
    
    stats.recordTest(defaultAccel == 10.0f, "デフォルト加速度値");
    stats.recordTest(defaultDecel == 15.0f, "デフォルト減速度値");
    
    // 値の設定と取得
    sim->setAgentMaxAcceleration(agentId, 5.0f);
    sim->setAgentMaxDeceleration(agentId, 8.0f);
    
    stats.recordTest(sim->getAgentMaxAcceleration(agentId) == 5.0f, "加速度設定・取得");
    stats.recordTest(sim->getAgentMaxDeceleration(agentId) == 8.0f, "減速度設定・取得");
    
    delete sim;
}

// テスト2: 加速度制限テスト
void testAccelerationLimit(TestStats& stats) {
    std::cout << "\n=== 加速度制限テスト ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.1f);  // 100ms
    
    // デフォルト設定を行う
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    size_t agentId = sim->addAgent(Vector3(0, 0, 0));
    sim->setAgentMaxAcceleration(agentId, 5.0f);  // 5 units/s²
    sim->setAgentMaxDeceleration(agentId, 8.0f);  // 8 units/s²
    
    // 初期速度を設定（静止状態）
    sim->setAgentVelocity(agentId, Vector3(0, 0, 0));
    
    // 大きな希望速度を設定（加速度制限をテスト）
    sim->setAgentPrefVelocity(agentId, Vector3(10.0f, 0, 0));
    
    // 1ステップ実行
    sim->doStep();
    
    Vector3 velocity = sim->getAgentVelocity(agentId);
    
    // 加速度制限により、速度は制限される
    // 期待値: maxAcceleration * timeStep = 5.0 * 0.1 = 0.5 units/s
    float expectedSpeed = 5.0f * 0.1f;  // 0.5 units/s
    
    stats.recordTest(isFloatNear(abs(velocity), expectedSpeed, 0.1f), 
                     "加速度制限による速度制限");
    
    delete sim;
}

// テスト3: 減速度制限テスト
void testDecelerationLimit(TestStats& stats) {
    std::cout << "\n=== 減速度制限テスト ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.1f);  // 100ms
    
    // デフォルト設定を行う
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    size_t agentId = sim->addAgent(Vector3(0, 0, 0));
    sim->setAgentMaxAcceleration(agentId, 5.0f);  // 5 units/s²
    sim->setAgentMaxDeceleration(agentId, 8.0f);  // 8 units/s²
    
    // 高速で移動中の状態を設定
    sim->setAgentVelocity(agentId, Vector3(10.0f, 0, 0));
    
    // 急停止を要求（減速度制限をテスト）
    sim->setAgentPrefVelocity(agentId, Vector3(0, 0, 0));
    
    Vector3 initialVelocity = sim->getAgentVelocity(agentId);
    
    // 1ステップ実行
    sim->doStep();
    
    Vector3 finalVelocity = sim->getAgentVelocity(agentId);
    
    // 速度変化量を計算
    float velocityChange = abs(finalVelocity - initialVelocity);
    float maxAllowedChange = 8.0f * 0.1f;  // maxDeceleration * timeStep = 0.8 units/s
    
    stats.recordTest(velocityChange <= maxAllowedChange + 0.1f, 
                     "減速度制限による速度変化制限");
    
    // 完全停止していないことを確認（減速度制限により）
    stats.recordTest(abs(finalVelocity) > 0.1f, 
                     "減速度制限により即座に停止しない");
    
    delete sim;
}

// テスト4: 制限内動作テスト
void testWithinLimits(TestStats& stats) {
    std::cout << "\n=== 制限内動作テスト ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.1f);
    
    // デフォルト設定を行う
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    size_t agentId = sim->addAgent(Vector3(0, 0, 0));
    sim->setAgentMaxAcceleration(agentId, 10.0f);  // 大きな制限値
    sim->setAgentMaxDeceleration(agentId, 10.0f);
    
    // 小さな速度変化を設定
    sim->setAgentVelocity(agentId, Vector3(1.0f, 0, 0));
    sim->setAgentPrefVelocity(agentId, Vector3(1.5f, 0, 0));  // 小さな加速
    
    // 1ステップ実行
    sim->doStep();
    
    Vector3 velocity = sim->getAgentVelocity(agentId);
    
    // 制限内なので、ほぼ希望速度に近い値になるはず
    stats.recordTest(isVectorNear(velocity, Vector3(1.5f, 0, 0), 0.2f), 
                     "制限内での正常動作");
    
    delete sim;
}

// テスト5: 複数エージェントシミュレーション
void testMultiAgentScenario(TestStats& stats) {
    std::cout << "\n=== 複数エージェントシミュレーション ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.1f);
    
    // デフォルト設定を行う
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    // 異なる加速度制限を持つ3つのエージェントを作成
    std::vector<size_t> agents;
    std::vector<float> accelLimits = {2.0f, 5.0f, 10.0f};
    
    for (int i = 0; i < 3; i++) {
        size_t agentId = sim->addAgent(Vector3(i * 5.0f, 0, 0));
        sim->setAgentMaxAcceleration(agentId, accelLimits[i]);
        sim->setAgentMaxDeceleration(agentId, accelLimits[i] * 1.5f);
        sim->setAgentVelocity(agentId, Vector3(0, 0, 0));
        sim->setAgentPrefVelocity(agentId, Vector3(10.0f, 0, 0));  // 全て同じ希望速度
        agents.push_back(agentId);
    }
    
    // 複数ステップ実行
    for (int step = 0; step < 5; step++) {
        sim->doStep();
    }
    
    // 各エージェントの速度を確認
    std::vector<float> finalSpeeds;
    for (size_t agentId : agents) {
        Vector3 velocity = sim->getAgentVelocity(agentId);
        finalSpeeds.push_back(abs(velocity));
    }
    
    // 加速度制限が小さいエージェントほど速度が小さいはず
    stats.recordTest(finalSpeeds[0] < finalSpeeds[1], 
                     "異なる加速度制限による速度差(1)");
    stats.recordTest(finalSpeeds[1] < finalSpeeds[2], 
                     "異なる加速度制限による速度差(2)");
    
    std::cout << "エージェント速度: ";
    for (int i = 0; i < 3; i++) {
        std::cout << finalSpeeds[i] << " ";
    }
    std::cout << std::endl;
    
    delete sim;
}

// テスト6: 実用的なシナリオテスト
void testRealWorldScenario(TestStats& stats) {
    std::cout << "\n=== 実用的シナリオテスト ===" << std::endl;
    
    RVOSimulator* sim = new RVOSimulator();
    sim->setTimeStep(0.05f);  // 50ms - より細かいタイムステップ
    
    // デフォルト設定を行う
    sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f, Vector3());
    
    // 人間の歩行をシミュレート
    size_t humanId = sim->addAgent(Vector3(0, 0, 0));
    sim->setAgentMaxAcceleration(humanId, 2.0f);  // 人間的な加速度
    sim->setAgentMaxDeceleration(humanId, 3.0f);  // 人間的な減速度
    sim->setAgentMaxSpeed(humanId, 1.5f);         // 歩行速度
    
    // 車両をシミュレート
    size_t vehicleId = sim->addAgent(Vector3(10, 0, 0));
    sim->setAgentMaxAcceleration(vehicleId, 3.0f);  // 車両の加速度
    sim->setAgentMaxDeceleration(vehicleId, 5.0f);  // 車両の減速度
    sim->setAgentMaxSpeed(vehicleId, 10.0f);        // 車両速度
    
    // 目標に向かって移動
    Vector3 humanGoal(20, 0, 0);
    Vector3 vehicleGoal(-10, 0, 0);
    
    std::vector<float> humanSpeeds;
    std::vector<float> vehicleSpeeds;
    
    // シミュレーション実行（2秒間、40ステップ）
    for (int step = 0; step < 40; step++) {
        // 目標への方向を計算
        Vector3 humanPos = sim->getAgentPosition(humanId);
        Vector3 vehiclePos = sim->getAgentPosition(vehicleId);
        
        Vector3 humanDir = normalize(humanGoal - humanPos);
        Vector3 vehicleDir = normalize(vehicleGoal - vehiclePos);
        
        // 希望速度を設定
        sim->setAgentPrefVelocity(humanId, humanDir * sim->getAgentMaxSpeed(humanId));
        sim->setAgentPrefVelocity(vehicleId, vehicleDir * sim->getAgentMaxSpeed(vehicleId));
        
        sim->doStep();
        
        // 速度を記録
        humanSpeeds.push_back(abs(sim->getAgentVelocity(humanId)));
        vehicleSpeeds.push_back(abs(sim->getAgentVelocity(vehicleId)));
    }
    
    // スムーズな加速カーブを確認
    bool humanSmoothAccel = true;
    bool vehicleSmoothAccel = true;
    
    for (int i = 1; i < 10; i++) {  // 最初の10ステップで判定
        if (humanSpeeds[i] < humanSpeeds[i-1]) humanSmoothAccel = false;
        if (vehicleSpeeds[i] < vehicleSpeeds[i-1]) vehicleSmoothAccel = false;
    }
    
    stats.recordTest(humanSmoothAccel, "人間エージェントのスムーズな加速");
    stats.recordTest(vehicleSmoothAccel, "車両エージェントのスムーズな加速");
    
    // 最終速度が制限内であることを確認
    stats.recordTest(humanSpeeds.back() <= sim->getAgentMaxSpeed(humanId) + 0.1f, 
                     "人間エージェントの最大速度制限");
    stats.recordTest(vehicleSpeeds.back() <= sim->getAgentMaxSpeed(vehicleId) + 0.1f, 
                     "車両エージェントの最大速度制限");
    
    std::cout << "人間最終速度: " << humanSpeeds.back() << " units/s" << std::endl;
    std::cout << "車両最終速度: " << vehicleSpeeds.back() << " units/s" << std::endl;
    
    delete sim;
}

int main() {
    std::cout << "=== RVO2-3D 加速度制限機能テスト ===" << std::endl;
    
    TestStats stats;
    
    try {
        testAPI(stats);
        testAccelerationLimit(stats);
        testDecelerationLimit(stats);
        testWithinLimits(stats);
        testMultiAgentScenario(stats);
        testRealWorldScenario(stats);
    } catch (const std::exception& e) {
        std::cout << "テスト実行中にエラーが発生しました: " << e.what() << std::endl;
        return 1;
    }
    
    stats.printSummary();
    
    if (stats.passedTests == stats.totalTests) {
        std::cout << "\n🎉 すべてのテストが成功しました！" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ 一部のテストが失敗しました。" << std::endl;
        return 1;
    }
} 