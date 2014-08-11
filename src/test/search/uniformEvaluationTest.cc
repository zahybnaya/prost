#include <vector>
#include <map>
#include <string>
#include "../gtest/gtest.h"
#include "../../search/uniform_evaluation_search.h"
#include "../../search/parser.h"
#include "../../search/prost_planner.h"

using std::vector;
using std::map;
using std::string;

// Tests the estimateQValuesFunction when no further parameters are given, i.e.
// the initial value was not set by hand.
TEST(UniformEvaluationTest, estimateQValuesStandard) {
    State dummyState;
    // Actions with index = i are applicable, actions with index = -1 are not
    // applicable, otherwise an action is unreasonable, see
    // SearchEngine::getApplicableActions()
    int actionIndices[] = {
        -1, 1, 2, 1, 4, -1, -1, 7, 8, 7, 10
    };
    vector<int> const actionsToExpand(actionIndices, actionIndices +
            sizeof(actionIndices) / sizeof(int));
    vector<double> qValues(sizeof(actionIndices), -1);
    UniformEvaluationSearch uniSearch;
    uniSearch.estimateQValues(dummyState, actionsToExpand, qValues);
    // Reasonable actions should be initialized with  0.0, since that is the
    // initial "initial value"
    ASSERT_DOUBLE_EQ(-1, qValues[0]);
    ASSERT_DOUBLE_EQ(0, qValues[1]);
    ASSERT_DOUBLE_EQ(0, qValues[2]);
    ASSERT_DOUBLE_EQ(-1, qValues[3]);
    ASSERT_DOUBLE_EQ(0, qValues[4]);
    ASSERT_DOUBLE_EQ(-1, qValues[5]);
    ASSERT_DOUBLE_EQ(-1, qValues[6]);
    ASSERT_DOUBLE_EQ(0, qValues[7]);
    ASSERT_DOUBLE_EQ(0, qValues[8]);
    ASSERT_DOUBLE_EQ(-1, qValues[9]);
    ASSERT_DOUBLE_EQ(0, qValues[10]);
}

// Tests Q-value estimation with a set initial Q-value.
TEST(UniformEvaluationTest, estimateQValuesWithSetValue) {
    State dummyState;
    int actionIndices[] = {
        -1, 1, 2, 1, 4, -1, -1, 7, 8, 7, 10
    };
    vector<int> const actionsToExpand(actionIndices, actionIndices +
            sizeof(actionIndices) / sizeof(int));
    vector<double> qValues(sizeof(actionIndices), -1);
    UniformEvaluationSearch uniSearch;
    uniSearch.setInitialValue(-2.5);
    uniSearch.estimateQValues(dummyState, actionsToExpand, qValues);
    ASSERT_DOUBLE_EQ(-1, qValues[0]);
    ASSERT_DOUBLE_EQ(-2.5, qValues[1]);
    ASSERT_DOUBLE_EQ(-2.5, qValues[2]);
    ASSERT_DOUBLE_EQ(-1, qValues[3]);
    // Some quick tests to check for other values
    // Positive Value
    uniSearch.setInitialValue(5);
    uniSearch.estimateQValues(dummyState, actionsToExpand, qValues);
    ASSERT_DOUBLE_EQ(5, qValues[1]);

    // Min and Max Values
    uniSearch.setInitialValue(std::numeric_limits<double>::max());
    uniSearch.estimateQValues(dummyState, actionsToExpand, qValues);
    ASSERT_DOUBLE_EQ(std::numeric_limits<double>::max(), qValues[1]);

    uniSearch.setInitialValue(-std::numeric_limits<double>::max());
    uniSearch.estimateQValues(dummyState, actionsToExpand, qValues);
    ASSERT_DOUBLE_EQ(-std::numeric_limits<double>::max(), qValues[1]);
}

// A trivial test for state value estimation.
TEST(UniformEvaluationTest, estimateStateValue) {
    State dummyState;
    UniformEvaluationSearch uniSearch;
    double stateValue = 0.0;
    uniSearch.setInitialValue(1.0);
    uniSearch.estimateStateValue(dummyState, stateValue);
    ASSERT_DOUBLE_EQ(1.0, stateValue);
}


// Tests the different string parameters.
TEST(UniformEvaluationTest, testSetValueFromString) {
    UniformEvaluationSearch uniSearch;
    State dummyState;
    string param = "-val";
    string value = "1.25";
    uniSearch.setValueFromString(param, value);
    double stateValue = 0.0;
    uniSearch.estimateStateValue(dummyState, stateValue);
    ASSERT_DOUBLE_EQ(1.25, stateValue);
    value = "INFTY";

    // Dummies to simulate a rewardCPF
    LogicalExpression* dummyExp = NULL;
    RewardFunction dummyCPF(dummyExp, 0, -2.0, 3.0, true);
    SearchEngine::rewardCPF = &dummyCPF;
    uniSearch.setValueFromString(param, value);
    uniSearch.estimateStateValue(dummyState, stateValue);
    ASSERT_DOUBLE_EQ(3.0, stateValue);
}
