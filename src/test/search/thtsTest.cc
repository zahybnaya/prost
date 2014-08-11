#include "../gtest/gtest.h"
#include "../../search/thts.h"
#include "../../search/prost_planner.h"
#include "../../search/parser.h"
#include "../../search/mc_uct_search.h"

using std::string;
using std::vector;
using std::map;

class THTSTestSearch : public MCUCTSearch {
public:
    // Wrapper functions to access protected functions
    void wrapInitStep(State const& _rootState) {
        initStep(_rootState);
    }

    void wrapInitializeDecisionNode(MCUCTNode* node) {
        initializeDecisionNode(node);
    }

    double wrapVisitDecisionNode(MCUCTNode* node) {
        return visitDecisionNode(node);
    }

    MCUCTNode* wrapGetSearchNode() {
        return getSearchNode();
    }

    int getMaxLockDepth() {
        return maxLockDepth;
    }

    void setMaxLockDepth(int _depth) {
        maxLockDepth = _depth;
    }

};

class THTSTest : public testing::Test {
protected:

    THTSTest() {
        string domainName = "crossing_traffic";
        string problemFileName =
            "../test/testdomains/" + domainName + "_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT -i [Uniform]]]";
        planner = new ProstPlanner(plannerDesc);

    }

    // Tests accessing private members
    void testInitializeDecisionNodeWhereBackupDepthChanges() {
        THTSTestSearch search;
        UniformEvaluationSearch* _initializer = new UniformEvaluationSearch();
        search.setInitializer(_initializer);
        vector<double> stateVector;
        for (int i = 0; i < State::numberOfDeterministicStateFluents; ++i) {
            stateVector.push_back(0);
        }
        for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
            stateVector.push_back(0);
        }
        int varIndex = stateVariableIndices["robot-at(x1, y1)"];
        stateVector[varIndex] = 1.0;

        State lowDepthState = State(stateVector, 25);
        State::calcStateFluentHashKeys(lowDepthState);
        State::calcStateHashKey(lowDepthState);

        // Initialize search with the initial state
        search.wrapInitStep(SearchEngine::initialState);
        search.setMaxLockDepth(40);
        // Simulate that the search is already deeper in the tree
        search.currentStateIndex = 25;
        search.states[search.currentStateIndex] = lowDepthState;
        MCUCTNode* node = search.wrapGetSearchNode();
        search.wrapInitializeDecisionNode(node);
        ASSERT_EQ(25, search.getMaxLockDepth());

        // For any other call in the same trial the lock depth stays the same
        stateVector[varIndex] = 0.0;
        varIndex = stateVariableIndices["robot-at(x2, y2)"];
        stateVector[varIndex] = 1.0;

        State nextDepthState = State(stateVector, 24);
        State::calcStateFluentHashKeys(nextDepthState);
        State::calcStateHashKey(nextDepthState);
        search.currentStateIndex = 24;
        search.states[search.currentStateIndex] = nextDepthState;
        search.wrapInitializeDecisionNode(node);
        ASSERT_EQ(25, search.getMaxLockDepth());

    }

    void testCorrectNumberOfInitializedDecisionNodes() {
        THTSTestSearch search;
        UniformEvaluationSearch* _initializer = new UniformEvaluationSearch();
        search.setInitializer(_initializer);
        // Set the actual state and index to the intial state and initialize the
        // node
        search.wrapInitStep(SearchEngine::initialState);
        MCUCTNode* node = search.wrapGetSearchNode();
        // Note that this node is not the same as the current root node 
        search.wrapInitializeDecisionNode(node);
        // Thus we should get one more initialized decision node
        ASSERT_EQ(1, search.initializedDecisionNodes);
        // Now we call the method on the correct root node, thus nothing should
        // change
        search.wrapInitializeDecisionNode(search.currentRootNode);
        ASSERT_EQ(1, search.initializedDecisionNodes);

    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
};


// What should happen, when we initialize a decision node?
// - If the node is a reward lock, just set reward lock flag and return
TEST_F(THTSTest, testInitializeDecisionNodeWithRewardLockNode) {
    THTSTestSearch search;
    // Create a state that is a reward lock and set it as root. We use the
    // crossing_traffic instance, where the state when the robot has reached a
    // goal is already a reward lock
    int robotIsAtGoalPosIndex = stateVariableIndices["robot-at(x3, y3)"];
    vector<double> stateVector;
    for (int i = 0 ; i < State::numberOfDeterministicStateFluents; ++i) {
        stateVector.push_back(0);
    }
    for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
        stateVector.push_back(0);
    }
    stateVector[robotIsAtGoalPosIndex] = 1.0;
    State stateWithRewardLock(stateVector, 5);
    State::calcStateFluentHashKeys(stateWithRewardLock);
    State::calcStateHashKey(stateWithRewardLock);
    // Set the actual state and index to the correct state and initialize the
    // node
    search.wrapInitStep(stateWithRewardLock);
    MCUCTNode* node = search.wrapGetSearchNode();
    search.wrapInitializeDecisionNode(node);
    ASSERT_TRUE(node->isARewardLock());
    ASSERT_EQ(0, node->children.size());
}

// - If the maximum backupLock depth is still the same as the max search depth,
// it should get set to the actual depth of the search.
TEST_F(THTSTest, testInitializeDecisionNodeWhereBackupDepthChanges) {
    testInitializeDecisionNodeWhereBackupDepthChanges();
}

// - The node should have as many children as there are applicable actions.
TEST_F(THTSTest, testInitializeDecisionNodeCorrectApplicableActions) {
    THTSTestSearch search;
    UniformEvaluationSearch* _initializer = new UniformEvaluationSearch();
    search.setInitializer(_initializer);
    // Set the actual state and index to the intial state and initialize the
    // node
    search.wrapInitStep(SearchEngine::initialState);
    MCUCTNode* node = search.wrapGetSearchNode();
    search.wrapInitializeDecisionNode(node);
    // For crossing traffic there should be initially 
    // 2 applicable actions (since we start in a corner) + noop
    int initializedActions = 0;
    for (int childIndex = 0; childIndex < node->children.size(); childIndex++) {
        if(node->children[childIndex]) {
            initializedActions++;
        }
    }
    ASSERT_EQ(3, initializedActions); 
}

// A simple test to assure that the number of initialized nodes is tracked
TEST_F(THTSTest, testCorrectNumberOfInitializedDecisionNodes) {
    testCorrectNumberOfInitializedDecisionNodes();
}


TEST_F(THTSTest, testVisitDecisionNodeWithRewardLock) {
    THTSTestSearch search;
    // Create a state that is a reward lock and set it as root. We use the
    // crossing_traffic instance, where the state when the robot has reached a
    // goal is already a reward lock
    int robotIsAtGoalPosIndex = stateVariableIndices["robot-at(x3, y3)"];
    vector<double> stateVector;
    for (int i = 0; i < State::numberOfDeterministicStateFluents; ++i) {
        stateVector.push_back(0);
    }
    for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
        stateVector.push_back(0);
    }
    stateVector[robotIsAtGoalPosIndex] = 1.0;
    State stateWithRewardLock = State(stateVector, 5);
    State::calcStateFluentHashKeys(stateWithRewardLock);
    State::calcStateHashKey(stateWithRewardLock);

    // Set the actual state and index to the correct state and initialize the
    // node
    search.wrapInitStep(stateWithRewardLock);
    MCUCTNode* node = search.wrapGetSearchNode();
    search.setCurrentRootNode(node);
    // The reward is calculated by 
    // reward + futReward * (ignoredSteps + currentStateIndex).
    ASSERT_DOUBLE_EQ(0, search.wrapVisitDecisionNode(node));

}
