#include "../gtest/gtest.h"
#include "../../search/thts.h"
#include "../../search/prost_planner.h"
#include "../../search/parser.h"
#include "../../search/mc_uct_search.h"
using std::string;
using std::vector;
using std::map;
using std::set;
using std::numeric_limits;

class EvaluateTest : public testing::Test {
protected:
    EvaluateTest() {
        string domainName = "crossing_traffic";
        string problemFileName = "../test/testdomains/"+domainName+"_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);
        // Create an Action State dummy, since we don't need one
        vector<int> vecDummy;
        vector<ActionFluent*> scheduledActionFluents;
        vector<DeterministicEvaluatable*> actionPreconditions;
        ActionState tmp(0, vecDummy, scheduledActionFluents, actionPreconditions);
        actionDummy = &tmp;

        // Create two states which we often use 
        vector<double> stateVector;
        string fluentName = "obstacle-at(x2, y2)";
        for (int i = 0; i < State::numberOfDeterministicStateFluents; ++i) {
            stateVector.push_back(0);
        }
        for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
            stateVector.push_back(0);
        }
        int varIndex = stateVariableIndices[fluentName];
        stateVector[varIndex] = 1.0;

        // Create two states
        x2y2True = State(stateVector, 25);
        stateVector[varIndex] = 0;
        x2y2False = State(stateVector, 25);
        State::calcStateFluentHashKeys(x2y2True);
        State::calcStateHashKey(x2y2True);
        State::calcStateFluentHashKeys(x2y2False);
        State::calcStateHashKey(x2y2False);
        
        // index of the state variable
        fluentName = "obstacle-at(x2, y2)";
        for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
            if (SearchEngine::stateFluents[i]->name == fluentName) {
                stateIndex = i;
                break;   
            }
        }
    }

    // Clear result after every test
    virtual void TearDown() {
        result = 0;
    }

    State x2y2True;
    State x2y2False;
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    State dummyState;
    ActionState* actionDummy;
    int stateIndex;
    string fluentName;
    double result;
    string s;
};

// Tests the evaluation of a deterministic state fluent with boolean values
TEST_F(EvaluateTest, testEvalBooleanDeterministicStateFluent) {
    // Get the state fluent which corresponds to obstacle-at(x2,y2)
    LogicalExpression* fluent = SearchEngine::stateFluents[stateIndex];
    // Test evaluation with one value 1
    fluent->evaluate(result, x2y2True, *actionDummy);
    ASSERT_DOUBLE_EQ(1.0, result);
    result = 0;
    // Test evaluation with one value 0
    fluent->evaluate(result, x2y2False, *actionDummy);
    ASSERT_DOUBLE_EQ(0.0, result);
    result = 0;
}

// Tests the evaluation of a probabilistic state fluent with boolean values
TEST_F(EvaluateTest, testEvalBooleanProbabilisticStateFluent) {
    vector<double> stateVector;
    fluentName = "obstacle-at(x3, y2)";
    for (int i = 0; i < State::numberOfDeterministicStateFluents; ++i) {
        stateVector.push_back(0);
    }
    for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
        stateVector.push_back(0);
    }
    int varIndex = stateVariableIndices[fluentName];
    stateVector[varIndex] = 1.0;

    // Create two states
    State x3y2True = State(stateVector, 25);
    stateVector[varIndex] = 0;
    State x3y2False = State(stateVector, 25);
    State::calcStateFluentHashKeys(x3y2True);
    State::calcStateHashKey(x3y2True);
    State::calcStateFluentHashKeys(x3y2False);
    State::calcStateHashKey(x3y2False);
   
    // Get the state fluent which corresponds to the name
    StateFluent* fluent;
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::stateFluents[i]->name == fluentName) {
            fluent = SearchEngine::stateFluents[i];
            break;   
        }
    }

    // Test evaluation with one value 1
    fluent->evaluate(result, x3y2True, *actionDummy);
    ASSERT_DOUBLE_EQ(1.0, result);
    result = 0;
    // Test evaluation with one value 0
    fluent->evaluate(result, x3y2False, *actionDummy);
    ASSERT_DOUBLE_EQ(0.0, result);
    result = 0;
}

// Tests the evaluation of an action fluent when there are no
// concurrent actions
TEST_F(EvaluateTest, testEvalActionFluentSingleAction) {
    // Get corresponding action state 
    fluentName = "move-west";
    ActionState* actionState;
    for (size_t i = 0; i < SearchEngine::actionStates.size();i++) {
        if (SearchEngine::actionStates[i].scheduledActionFluents.size() == 1 &&
                SearchEngine::actionStates[i].scheduledActionFluents[0]->name 
                == fluentName) {
            actionState = &SearchEngine::actionStates[i];
        }
    }

    // Get the action fluent which corresponds to the name
    ActionFluent* fluent;
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }

    // Since the actionFluent corresponds to the action state it should evaluate
    // to 1
    fluent->evaluate(result, dummyState, *actionState);
    ASSERT_DOUBLE_EQ(1.0, result);
    result = 0;

    fluentName = "move-east";
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }
    // Move east is not in the action state, therefore it should evaluate to 0
    result = 0;
    fluent->evaluate(result, dummyState, *actionState);
    ASSERT_DOUBLE_EQ(0, result);
    result = 0;

}

// Tests the evaluation of an action fluent with concurrent actions
TEST_F(EvaluateTest, testEvalActionFluentConcurrentActions) {
    string domainName = "earth_observation";
    string problemFileName = "../test/testdomains/" + 
        domainName + "_inst_mdp__03";
    Parser parser(problemFileName);
    parser.parseTask(stateVariableIndices, stateVariableValues);

    // Get action state where slew(east) and take-image is possible, 
    // which is the only one with two scheduledActionFluents
    ActionState* actionState;
    for (size_t i = 0; i < SearchEngine::actionStates.size();i++) {
        if (SearchEngine::actionStates[i].scheduledActionFluents.size() 
                == 2) {
            actionState = &SearchEngine::actionStates[i];
        }
    }

    fluentName = "take-image";
    // Get the action fluent which corresponds to the name
    ActionFluent* fluent;
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }
    // Since the actionFluent corresponds to the action state it should evaluate
    // to 1
    fluent->evaluate(result, dummyState, *actionState);
    ASSERT_DOUBLE_EQ(1.0, result);
    result = 0;

    fluentName = "slew(@north-east)";
    // Get the action fluent which corresponds to the name
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }
    // This action is not in the action state
    result = 0;
    fluent->evaluate(result, dummyState, *actionState);
    ASSERT_DOUBLE_EQ(0, result);
    result = 0;
}

// Tests evaluation of a constant
TEST_F(EvaluateTest, testEvaluateConstant) {
    s = "$c(2.0)";
    LogicalExpression* constant = LogicalExpression::createFromString(s);

    constant->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(2.0, result);
    s = "$c(-1.5)";
    LogicalExpression* negConstant = LogicalExpression::createFromString(s);
    result = 0;
    negConstant->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1.5, result);
}

// Tests evaluation of conjunctions which are either 1 or 0
TEST_F(EvaluateTest, testEvalConjunctionWithSingleResult) {
    s = "and($c(1) $c(0) $c(0))";
    LogicalExpression* conjunct = LogicalExpression::createFromString(s);

    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "and($c(0) $c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "and($c(0) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "and($c(0))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "and($c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "and($c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "and($c(1) $c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result = 0;
    conjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation of disjunctions which are either 1 or 0
TEST_F(EvaluateTest, testEvalDisjunctionWithSingleResult) {
    s = "or($c(1) $c(0) $c(0))";
    LogicalExpression* disjunct = LogicalExpression::createFromString(s);
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "or($c(0) $c(1) $c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "or($c(0) $c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "or($c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "or($c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "or($c(0) $c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "or($c(0) $c(0) $c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result = 0;
    disjunct->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);
}

// Tests evaluation with EqualsExpression
TEST_F(EvaluateTest, testEvalEqualsWithSingleResult) {
    s = "==($c(1) $c(0))";
    LogicalExpression* equal = LogicalExpression::createFromString(s);
    equal->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "==($c(1) $c(1))";
    equal = LogicalExpression::createFromString(s);
    result = 0;
    equal->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "==($c(0) $c(0))";
    equal = LogicalExpression::createFromString(s);
    result = 0;
    equal->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "==($c(-1) $c(1))";
    equal = LogicalExpression::createFromString(s);
    result = 0;
    equal->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "==($c(-2.5) $c(-2.5))";
    equal = LogicalExpression::createFromString(s);
    result = 0;
    equal->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation with Greater Expression
TEST_F(EvaluateTest, testEvalGreaterWithSingleResult) {
    s = ">($c(1) $c(0))";
    LogicalExpression* greater = LogicalExpression::createFromString(s);
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = ">($c(1) $c(1))";
    greater = LogicalExpression::createFromString(s);
    result = 0;
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = ">($c(0) $c(0))";
    greater = LogicalExpression::createFromString(s);
    result = 0;
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = ">($c(-1) $c(1))";
    greater = LogicalExpression::createFromString(s);
    result = 0;
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = ">($c(-2.5) $c(-2.5))";
    greater = LogicalExpression::createFromString(s);
    result = 0;
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = ">($c(2.5) $c(-2.5))";
    greater = LogicalExpression::createFromString(s);
    result = 0;
    greater->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation with Lower Expression
TEST_F(EvaluateTest, testEvalLowerWithSingleResult) {
    s = "<($c(1) $c(0))";
    LogicalExpression* lower = LogicalExpression::createFromString(s);
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<($c(1) $c(1))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<($c(0) $c(0))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<($c(-1) $c(1))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "<($c(-2.5) $c(-2.5))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<($c(2.5) $c(-2.5))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<($c(-2.5) $c(2.5))";
    lower = LogicalExpression::createFromString(s);
    result = 0;
    lower->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation with GreaterEqual Expression
TEST_F(EvaluateTest, testEvalGreaterEqualWithSingleResult) {
    s = ">=($c(1) $c(0))";
    LogicalExpression* greaterEqual = LogicalExpression::createFromString(s);

    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = ">=($c(1) $c(1))";
    greaterEqual = LogicalExpression::createFromString(s);
    result = 0;
    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = ">=($c(0) $c(0))";
    greaterEqual = LogicalExpression::createFromString(s);
    result = 0;
    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = ">=($c(-1) $c(1))";
    greaterEqual = LogicalExpression::createFromString(s);
    result = 0;
    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = ">=($c(-2.5) $c(-2.5))";
    greaterEqual = LogicalExpression::createFromString(s);
    result = 0;
    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = ">=($c(2.5) $c(-2.5))";
    greaterEqual = LogicalExpression::createFromString(s);
    result = 0;
    greaterEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation with LowerEqual Expression
TEST_F(EvaluateTest, testEvalLowerEqualWithSingleResult) {
    s = "<=($c(1) $c(0))";
    LogicalExpression* lowerEqual = LogicalExpression::createFromString(s);

    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "<=($c(1) $c(1))";
    lowerEqual = LogicalExpression::createFromString(s);
    result = 0;
    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "<=($c(0) $c(0))";
    lowerEqual = LogicalExpression::createFromString(s);
    result = 0;
    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "<=($c(-1) $c(1))";
    lowerEqual = LogicalExpression::createFromString(s);
    result = 0;
    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "<=($c(-2.5) $c(-2.5))";
    lowerEqual = LogicalExpression::createFromString(s);
    result = 0;
    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "<=($c(2.5) $c(-2.5))";
    lowerEqual = LogicalExpression::createFromString(s);
    result = 0;
    lowerEqual->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);
}

// Tests evaluation of simple additions
TEST_F(EvaluateTest, testEvalAdditionWithSingleResult) {
    s = "+($c(1) $c(0) $c(0))";
    LogicalExpression* addition = LogicalExpression::createFromString(s);

    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "+($c(0) $c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(2, result);

    s = "+($c(0) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "+($c(0))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);

    s = "+($c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);

    s = "+($c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(2, result);

    s = "+($c(1) $c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(3, result);

    s = "+($c(0) $c(0))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "+($c(-1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "+($c(-2.5) $c(-2.5))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-5, result);

    s = "+($c(2.5) $c(-2.5))";
    addition = LogicalExpression::createFromString(s);
    result = 0;
    addition->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);
}

// Tests evaluation of subtractions. Note that subtractions 
// are only defined for two expressions
TEST_F(EvaluateTest, testEvalSubtractionWithSingleResult) {
    s = "-($c(0) $c(1))";
    LogicalExpression* subtraction = LogicalExpression::createFromString(s);
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1, result);

    s = "-($c(1) $c(1))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);


    s = "-($c(0) $c(0))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "-($c(-1) $c(1))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-2, result);

    s = "-($c(-2.5) $c(-2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "-($c(2.5) $c(-2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(5, result);

    s = "-($c(-2.5) $c(2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result = 0;
    subtraction->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-5, result);
}

// Tests of multiplications. Note that multiplications 
// are only defined for two expressions
TEST_F(EvaluateTest, testEvalMultiplicationWithSingleResult) {
    s = "*($c(0) $c(1))";
    LogicalExpression* multiplication = LogicalExpression::createFromString(s);
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "*(*($c(1) $c(1)) $c(1))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "*($c(0) $c(0))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "*($c(-1) $c(1))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1, result);

    s = "*($c(-2.5) $c(-2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(6.25, result);

    s = "*($c(2.5) $c(-2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-6.25, result);

    s = "*($c(-2.5) $c(2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result = 0;
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-6.25, result);
}

// Tests evaluation of divisions. Note that divisions 
// are only defined for two expressions
TEST_F(EvaluateTest, testEvalDivisionWithSingleResult) {
    s = "/($c(0) $c(1))";
    LogicalExpression* division = LogicalExpression::createFromString(s);
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    s = "/($c(1) $c(1))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);


    s = "/($c(-1) $c(1))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1, result);

    s = "/($c(-2.5) $c(-2.5))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    s = "/($c(2.5) $c(-2.5))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1, result);

    s = "/($c(-2.5) $c(2.5))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-1, result);

    s = "/($c(-2) $c(5))";
    division = LogicalExpression::createFromString(s);
    result = 0;
    division->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(-0.4, result);
}

// Tests evaluation of negation
TEST_F(EvaluateTest, testEvalNegation) {
    // ~1 = 0
    s = "~($c(1))";
    LogicalExpression* negation = LogicalExpression::createFromString(s);
    result = 0;
    negation->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0, result);

    // ~0 = 1
    s = "~($c(0))";
    negation = LogicalExpression::createFromString(s);
    result = 0;
    negation->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);
}

// Tests evaluation of multiconditions
TEST_F(EvaluateTest, testEvalMultiCond) {
    // if (1) then (0.5) if(1) then (2)
    std::stringstream ss;
    ss << "switch( ($c(1) : $c(0.5)) ($c(1) : $c(2)))";
    string s = ss.str();
    LogicalExpression* multicond = LogicalExpression::createFromString(s);
    multicond->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(0.5, result);

    // if (0) then (0.5) else (2) = 2
    ss.str("");
    ss <<"switch( ($c(0) : $c(0.5)) ($c(1) : $c(2)))";
    s = ss.str();
    result = 0;
    multicond = LogicalExpression::createFromString(s);
    multicond->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(2, result);
}

// Tests evaluation of if's inside other conditions
TEST_F(EvaluateTest, testEvalNestedCond) {
    // equals 2 * 0.5 = 1
    s = "*(switch( ($c(0) : $c(1)) ($c(1) : $c(2)))"
        "switch( ($c(1) : $c(0.5)) ($c(1) : $c(2))))";
    LogicalExpression* multiplication = LogicalExpression::createFromString(s);
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(1, result);

    // equals 3 * 2 * 0.5 = 3
    s = "*(switch( ($c(0) : $c(1)) ($c(0) : $c(2)) ($c(1) : $c(3)))" 
        "*(switch( ($c(0) : $c(1)) ($c(1) : $c(2)))"
        "switch( ($c(1) : $c(0.5)) ($c(1) : $c(2)))))";
    multiplication = LogicalExpression::createFromString(s);
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(3, result);

    // equals 3 + 2 + 0.5 = 5.5
    s = "+(switch( ($c(0) : $c(1)) ($c(0) : $c(2)) ($c(1) : $c(3)))" 
        "switch( ($c(0) : $c(1)) ($c(1) : $c(2)))"
        "switch( ($c(1) : $c(0.5)) ($c(1) : $c(2))))";
    multiplication = LogicalExpression::createFromString(s);
    multiplication->evaluate(result, dummyState, *actionDummy);
    ASSERT_DOUBLE_EQ(5.5, result);
}

