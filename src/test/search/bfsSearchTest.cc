#include "../gtest/gtest.h"
#include "../../search/prost_planner.h"
#include "../../search/parser.h"
#include "../../search/breadth_first_search.h"

using std::string;
using std::map;
using std::vector;

class BFSTestSearch : public BreadthFirstSearch {
public:

    // Wrapper functions to access protected functions
    void wrapInitializeDecisionNodeChild(BFSNode* node,
            unsigned int const& actionIndex,
            double const& initialQValue) {
        initializeDecisionNodeChild(node, actionIndex, initialQValue);
    }

    void wrapBackupDecisionNodeLeaf(BFSNode* node, double const& immReward,
            double const& futureReward) {
        backupDecisionNodeLeaf(node, immReward, futureReward);
    }

    void wrapBackupDecisionNode(BFSNode* node, double const& immReward,
            double const& futureReward) {
        backupDecisionNode(node, immReward, futureReward);
    }

    void wrapBackupChanceNode(BFSNode* node, double const& futReward) {
        backupChanceNode(node, futReward);
    }

    int wrapSelectAction(BFSNode* node) {
        return selectAction(node);
    }

    void wrapInitStep(State const& _rootState) {
        initStep(_rootState);
    }

    bool getBackupLock() {
        return backupLock;
    }
};

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class BFSSearchTest : public testing::Test {
protected:
    virtual void SetUp() {
        // Parse elevator task
        string problemFileName = "../test/testdomains/elevators_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT]]";
        planner = new ProstPlanner(plannerDesc);

        // Initialize other variables
        qValue = 10.0;
        parent = new BFSNode();
        childOne = new BFSNode();
        childTwo = new BFSNode();
        childThree = new BFSNode();
        srand(1);
    }

    virtual void teardown() {
        delete planner;
        delete parent;
    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    double qValue;
    BFSNode* parent;
    BFSNode* childOne;
    BFSNode* childTwo;
    BFSNode* childThree;
};

// Tests the initialization of a decision node child
TEST_F(BFSSearchTest, testBFSInitializeDecisionNodeChild) {
    BFSTestSearch search;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    search.wrapInitializeDecisionNodeChild(parent, 0, 10);
    search.wrapInitializeDecisionNodeChild(parent, 1, -10);
    EXPECT_DOUBLE_EQ(400, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(400, parent->children[0]->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(-400, parent->children[1]->getExpectedFutureRewardEstimate());
    EXPECT_EQ(0, parent->getNumberOfVisits());
    EXPECT_EQ(0, parent->children[0]->getNumberOfVisits());
    EXPECT_EQ(0, parent->children[1]->getNumberOfVisits());
}

// Tests the outcome selection
TEST_F(BFSSearchTest, testBFSSelectOutcome) {
    // TODO: Write an actual test for this
}

// Tests backup of a decision node leaf
TEST_F(BFSSearchTest, testBFSBackupDecisionNodeLeaf) {
    BFSTestSearch search;
    // Create a node with futReward 0 without accessing private members
    parent->children.push_back(childOne);
    search.wrapInitializeDecisionNodeChild(parent, 0, 0);
    BFSNode* node = parent->children[0];
    search.wrapBackupDecisionNodeLeaf(node, 10, 20);
    EXPECT_DOUBLE_EQ(20, node->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(30, node->getExpectedRewardEstimate());
    ASSERT_TRUE(node->isSolved());
}

// Tests backup of a decision node
TEST_F(BFSSearchTest, testBFSBackupDecisionNode) {
    BFSTestSearch search;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);

    // Initialize children. Note that horizon is 40, thus the expected future
    // reward will be 0, 4, 2
    search.wrapInitializeDecisionNodeChild(parent, 0, 0);
    search.wrapInitializeDecisionNodeChild(parent, 1, 0.1);
    search.wrapInitializeDecisionNodeChild(parent, 2, 0.05);

    delete childOne;
    delete childTwo;
    delete childThree;

    // Assign correct children
    childOne = parent->children[0];
    childTwo = parent->children[1];
    childThree = parent->children[2];

    // Increase future reward of one child and backup parent
    search.wrapBackupDecisionNodeLeaf(childOne, 0, 10);
    search.wrapBackupDecisionNode(parent, 2, -5);
    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12, parent->getExpectedRewardEstimate());
    ASSERT_FALSE(search.getBackupLock());

    // Increase another child with a lower value, backupLock should get true
    search.wrapBackupDecisionNodeLeaf(childTwo, 0, 5);
    search.wrapBackupDecisionNode(parent, 2, -1);
    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12, parent->getExpectedRewardEstimate());
    ASSERT_TRUE(search.getBackupLock());
}

// Tests backup of a chance node
TEST_F(BFSSearchTest, testBFSBackupChanceNode) {
    // TODO Test case
}

// Tests action selection for breadth first search
TEST_F(BFSSearchTest, testBFSSelectAction) {
    BFSTestSearch search;
    search.wrapInitStep(SearchEngine::initialState);

    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);

    BFSNode* grandchildOne = new BFSNode();
    BFSNode* grandchildTwo = new BFSNode();

    childTwo->children.push_back(grandchildOne);
    childThree->children.push_back(grandchildTwo);


    //// TODO: Change the test so that we don't have to exploit private access of
    //// visits
    //int selectedActionIndex = search.wrapSelectAction(parent);
    //// There are 3 children, so we want the corresponding random number
    //int randomNumber = std::rand() % 3;
    //srand(1);
    //ASSERT_EQ(randomNumber, selectedActionIndex);
    //search.wrapBackupDecisionNodeLeaf(childOne, 0, 0);
    //// Now one child one is solved and should never get selected again
    //randomNumber = std::rand() % 2;
    //selectedActionIndex = search.wrapSelectAction(parent);
    //ASSERT_EQ(randomNumber, selectedActionIndex);

    //search.wrapBackupDecisionNode(childTwo, 0, 0);
    //selectedActionIndex = search.wrapSelectAction(parent);

    //ASSERT_EQ(2, selectedActionIndex);
    //search.wrapBackupDecisionNode(childThree, 0, 0);
    //selectedActionIndex = search.wrapSelectAction(parent);
    //// Note that child one is solved, therefore we select child two again
    //ASSERT_EQ(1, selectedActionIndex);

    // TODO: Write a test where not every index of the children is used. Right
    // now the method works for these cases, but the tests should be implemented
    // to preserve maintainability.
}
