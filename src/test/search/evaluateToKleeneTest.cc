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

class EvaluateToKleeneTest : public testing::Test {
protected:
    EvaluateToKleeneTest() {
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

        // Create two kleene states which we often use to test kleene states
        // with more than one value
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

        // Create two kleene states
        State x2y2True = State(stateVector, 25);
        stateVector[varIndex] = 0;
        State x2y2False = State(stateVector, 25);
        State::calcStateFluentHashKeys(x2y2True);
        State::calcStateHashKey(x2y2True);
        State::calcStateFluentHashKeys(x2y2False);
        State::calcStateHashKey(x2y2False);
        kleeneT = KleeneState(x2y2True);
        kleeneF = KleeneState(x2y2False);
        KleeneState::calcStateHashKey(kleeneT);
        KleeneState::calcStateFluentHashKeys(kleeneT);
        KleeneState::calcStateHashKey(kleeneF);
        KleeneState::calcStateFluentHashKeys(kleeneF);

        // index of the kleene state variable
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
        result.clear();
    }

    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    KleeneState dummyState;
    ActionState* actionDummy;
    KleeneState kleeneT;
    KleeneState kleeneF;
    int stateIndex;
    string fluentName;
    set<double> result;
};

// Tests the evaluation of a deterministic state fluent with boolean values
TEST_F(EvaluateToKleeneTest, testEvalBooleanDeterministicStateFluent) {
    // Get the state fluent which corresponds to obstacle-at(x2,y2)
    LogicalExpression* fluent = SearchEngine::stateFluents[stateIndex];
    // Test evaluation with one value 1
    fluent->evaluateToKleene(result, kleeneT, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_DOUBLE_EQ(1.0, *(result.begin()));
    result.clear();
    // Test evaluation with one value 0
    fluent->evaluateToKleene(result, kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_DOUBLE_EQ(0.0, *(result.begin()));
    result.clear();
    // Test evaluation with values 0 and 1
    fluent->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0.0) != result.end());
    ASSERT_TRUE(result.find(1.0) != result.end());
    result.clear();
}

// Tests the evaluation of a probabilistic state fluent with boolean values
TEST_F(EvaluateToKleeneTest, testEvalBooleanProbabilisticStateFluent) {
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

    // Create two kleene states
    State x3y2True = State(stateVector, 25);
    stateVector[varIndex] = 0;
    State x3y2False = State(stateVector, 25);
    State::calcStateFluentHashKeys(x3y2True);
    State::calcStateHashKey(x3y2True);
    State::calcStateFluentHashKeys(x3y2False);
    State::calcStateHashKey(x3y2False);
    KleeneState kleene1(x3y2True);
    KleeneState kleene2(x3y2False);

    KleeneState::calcStateHashKey(kleene1);
    KleeneState::calcStateFluentHashKeys(kleene1);
    KleeneState::calcStateHashKey(kleene2);
    KleeneState::calcStateFluentHashKeys(kleene2);

    // Get the state fluent which corresponds to the name
    StateFluent* fluent;
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::stateFluents[i]->name == fluentName) {
            fluent = SearchEngine::stateFluents[i];
            break;   
        }
    }

    // Test evaluation with one value 1
    fluent->evaluateToKleene(result, kleene1, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_DOUBLE_EQ(1.0, *(result.begin()));
    result.clear();
    // Test evaluation with one value 0
    fluent->evaluateToKleene(result, kleene2, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_DOUBLE_EQ(0.0, *(result.begin()));
    result.clear();
    // Test evaluation with values 0 and 1
    fluent->evaluateToKleene(result, kleene1||kleene2, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0.0) != result.end());
    ASSERT_TRUE(result.find(1.0) != result.end());
    result.clear();
}

// Tests the evaluation of an action fluent when there are no
// concurrent actions
TEST_F(EvaluateToKleeneTest, testEvalActionFluentSingleAction) {
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
    fluent->evaluateToKleene(result, dummyState, *actionState);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1.0) != result.end());
    result.clear();

    fluentName = "move-east";
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }
    // Move east is not in the action state, therefore it should evaluate to 0
    result.clear();
    fluent->evaluateToKleene(result, dummyState, *actionState);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0.0) != result.end());
    result.clear();

}

// Tests the evaluation of an action fluent with concurrent actions
TEST_F(EvaluateToKleeneTest, testEvalActionFluentConcurrentActions) {
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
    fluent->evaluateToKleene(result, dummyState, *actionState);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1.0) != result.end());
    result.clear();

    fluentName = "slew(@north-east)";
    // Get the action fluent which corresponds to the name
    for (size_t i = 0; i < SearchEngine::stateFluents.size(); i++) {
        if (SearchEngine::actionFluents[i]->name == fluentName) {
            fluent = SearchEngine::actionFluents[i];
            break;   
        }
    }
    // This action is not in the action state
    result.clear();
    fluent->evaluateToKleene(result, dummyState, *actionState);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0.0) != result.end());
    result.clear();
}

// Tests kleene evaluation of a constant
TEST_F(EvaluateToKleeneTest, testEvaluateConstant) {
    string s = "$c(2.0)";
    LogicalExpression* constant = LogicalExpression::createFromString(s);

    constant->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(2.0) != result.end());
    s = "$c(-1.5)";
    LogicalExpression* negConstant = LogicalExpression::createFromString(s);
    result.clear();
    negConstant->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1.5) != result.end());
}

// Tests kleene evaluation of conjunctions which are either 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalConjunctionWithSingleResult) {
    string s = "and($c(1) $c(0) $c(0))";
    LogicalExpression* conjunct = LogicalExpression::createFromString(s);

    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "and($c(0) $c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "and($c(0) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "and($c(0))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "and($c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "and($c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "and($c(1) $c(1) $c(1))";
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of conjunctions which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalConjunctionWithMultiResult) {
    // AND({0,1}, 0] = 0
    std::stringstream ss;
    ss <<"and($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* conjunct = LogicalExpression::createFromString(s);

    conjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"and($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // AND({0,1}, 1) = {0,1}
    ss.str("");
    ss <<"and($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"and($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    conjunct = LogicalExpression::createFromString(s);
    result.clear();
    conjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of disjunctions which are either 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalDisjunctionWithSingleResult) {
    string s = "or($c(1) $c(0) $c(0))";
    LogicalExpression* disjunct = LogicalExpression::createFromString(s);

    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "or($c(0) $c(1) $c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "or($c(0) $c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "or($c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "or($c(1))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "or($c(0) $c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "or($c(0) $c(0) $c(0))";
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of disjunctions which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalDisjunctionWithMultiResult) {
    // OR({0,1}, 0] = {0,1}
    std::stringstream ss;
    ss <<"or($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* disjunct = LogicalExpression::createFromString(s);

    disjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"or($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // OR({0,1}, 1) = 1
    ss.str("");
    ss <<"or($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"or($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    disjunct = LogicalExpression::createFromString(s);
    result.clear();
    disjunct->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation with EqualsExpression
TEST_F(EvaluateToKleeneTest, testEvalEqualsWithSingleResult) {
    // Simple tests
    string s = "==($c(1) $c(0))";
    LogicalExpression* equal = LogicalExpression::createFromString(s);

    equal->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "==($c(1) $c(1))";
    equal = LogicalExpression::createFromString(s);
    result.clear();
    equal->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "==($c(0) $c(0))";
    equal = LogicalExpression::createFromString(s);
    result.clear();
    equal->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "==($c(-1) $c(1))";
    equal = LogicalExpression::createFromString(s);
    result.clear();
    equal->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "==($c(-2.5) $c(-2.5))";
    equal = LogicalExpression::createFromString(s);
    result.clear();
    equal->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of equals which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalEqualsMultiResult) {
    // {0,1} == 0 = {0,1}
    std::stringstream ss;
    ss <<"==($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* equals = LogicalExpression::createFromString(s);

    equals->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"==($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    equals = LogicalExpression::createFromString(s);
    result.clear();
    equals->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // {0,1} == 1 = {0,1}
    ss.str("");
    ss <<"==($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    equals = LogicalExpression::createFromString(s);
    result.clear();
    equals->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"==($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    equals = LogicalExpression::createFromString(s);
    result.clear();
    equals->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());


    // {0,1} == 2 = 0
    ss.str("");
    ss <<"==($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    equals = LogicalExpression::createFromString(s);
    result.clear();
    equals->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation with Greater Expression
TEST_F(EvaluateToKleeneTest, testEvalGreaterWithSingleResult) {
    // Simple tests
    string s = ">($c(1) $c(0))";
    LogicalExpression* greater = LogicalExpression::createFromString(s);

    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = ">($c(1) $c(1))";
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = ">($c(0) $c(0))";
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = ">($c(-1) $c(1))";
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = ">($c(-2.5) $c(-2.5))";
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = ">($c(2.5) $c(-2.5))";
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of greater which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalGreaterMultiResult) {
    // {0,1} > 0 = {0,1}
    std::stringstream ss;
    ss <<">($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* greater = LogicalExpression::createFromString(s);

    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // 0 > {0,1} = 0
    ss.str("");
    ss <<">($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // {0,1} > 1 = 0
    ss.str("");
    ss <<">($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // 1 > {0,1} = {0,1}
    ss.str("");
    ss <<">($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());


    // {0,1} > 2 = 0
    ss.str("");
    ss <<">($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // 2 > {0,1} = 1
    ss.str("");
    ss <<">($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    greater = LogicalExpression::createFromString(s);
    result.clear();
    greater->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation with Lower Expression
TEST_F(EvaluateToKleeneTest, testEvalLowerWithSingleResult) {
    // Simple tests
    string s = "<($c(1) $c(0))";
    LogicalExpression* lower = LogicalExpression::createFromString(s);

    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "<($c(1) $c(1))";
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "<($c(0) $c(0))";
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "<($c(-1) $c(1))";
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "<($c(-2.5) $c(-2.5))";
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "<($c(2.5) $c(-2.5))";
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of lower which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalLowerMultiResult) {
    // {0,1} < 0 = 0
    std::stringstream ss;
    ss <<"<($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* lower = LogicalExpression::createFromString(s);

    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // 0 < {0,1} = {0,1}
    ss.str("");
    ss <<"<($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // {0,1} < 1 = {0,1}
    ss.str("");
    ss <<"<($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // 1 < {0,1} = 0
    ss.str("");
    ss <<"<($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());


    // {0,1} < 2 = 1
    ss.str("");
    ss <<"<($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // 2 < {0,1} = 0
    ss.str("");
    ss <<"<($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    lower = LogicalExpression::createFromString(s);
    result.clear();
    lower->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation with GreaterEqual Expression
TEST_F(EvaluateToKleeneTest, testEvalGreaterEqualWithSingleResult) {
    // Simple tests
    string s = ">=($c(1) $c(0))";
    LogicalExpression* greaterEqual = LogicalExpression::createFromString(s);

    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = ">=($c(1) $c(1))";
    greaterEqual = LogicalExpression::createFromString(s);
    result.clear();
    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = ">=($c(0) $c(0))";
    greaterEqual = LogicalExpression::createFromString(s);
    result.clear();
    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = ">=($c(-1) $c(1))";
    greaterEqual = LogicalExpression::createFromString(s);
    result.clear();
    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = ">=($c(-2.5) $c(-2.5))";
    greaterEqual = LogicalExpression::createFromString(s);
    result.clear();
    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = ">=($c(2.5) $c(-2.5))";
    greaterEqual = LogicalExpression::createFromString(s);
    result.clear();
    greaterEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of lowerEqual which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalGreaterEqualMultiResult) {
    // {0,1} >= 0 = 1
    std::stringstream ss;
    ss <<">=($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* lowerEqual = LogicalExpression::createFromString(s);

    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // 0 >= {0,1} = {0,1}
    ss.str("");
    ss <<">=($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // {0,1} >= 1 = {0,1}
    ss.str("");
    ss <<">=($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // 1 >= {0,1} = 1
    ss.str("");
    ss <<">=($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());


    // {0,1} >= 2 = 0
    ss.str("");
    ss <<">=($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // 2 >= {0,1} = 1
    ss.str("");
    ss <<">=($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation with LowerEqual Expression
TEST_F(EvaluateToKleeneTest, testEvalLowerEqualWithSingleResult) {
    // Simple tests
    string s = "<=($c(1) $c(0))";
    LogicalExpression* lowerEqual = LogicalExpression::createFromString(s);

    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "<=($c(1) $c(1))";
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "<=($c(0) $c(0))";
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "<=($c(-1) $c(1))";
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "<=($c(-2.5) $c(-2.5))";
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "<=($c(2.5) $c(-2.5))";
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of lowerEqual which can be 1 or 0
TEST_F(EvaluateToKleeneTest, testEvalLowerEqualMultiResult) {
    // {0,1} <= 0 = {0,1}
    std::stringstream ss;
    ss <<"<=($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* lowerEqual = LogicalExpression::createFromString(s);

    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // 0 <= {0,1} = 1
    ss.str("");
    ss <<"<=($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // {0,1} <= 1 = 1
    ss.str("");
    ss <<"<=($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // 1 <= {0,1} = {0,1}
    ss.str("");
    ss <<"<=($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());


    // {0,1} <= 2 = 1
    ss.str("");
    ss <<"<=($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // 2 <= {0,1} = 0
    ss.str("");
    ss <<"<=($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    lowerEqual = LogicalExpression::createFromString(s);
    result.clear();
    lowerEqual->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of simple additions
TEST_F(EvaluateToKleeneTest, testEvalAdditionWithSingleResult) {
    string s = "+($c(1) $c(0) $c(0))";
    LogicalExpression* addition = LogicalExpression::createFromString(s);

    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "+($c(0) $c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(2) != result.end());

    s = "+($c(0) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "+($c(0))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(0, result.size());

    s = "+($c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(0, result.size());

    s = "+($c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(2) != result.end());

    s = "+($c(1) $c(1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(3) != result.end());

    s = "+($c(0) $c(0))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "+($c(-1) $c(1))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "+($c(-2.5) $c(-2.5))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-5) != result.end());

    s = "+($c(2.5) $c(-2.5))";
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of additions on one multistate
TEST_F(EvaluateToKleeneTest, testEvalAdditionWithOneMultiState) {
    // {0,1} + 0 = {0,1}
    std::stringstream ss;
    ss <<"+($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* addition = LogicalExpression::createFromString(s);

    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"+($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());


    // {0,1} + 1) = {1,2}
    ss.str("");
    ss <<"+($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());

    // Reverse order test
    ss.str("");
    ss <<"+($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());

    // {0,1} + 2 = {2,3}
    ss.str("");
    ss <<"+($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(3) != result.end());

    // 2 + {0,1} = {2,3}
    ss.str("");
    ss <<"+($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);
    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(3) != result.end());
}

// Tests kleene evaluation of additions on multistates
TEST_F(EvaluateToKleeneTest, testEvalAdditionWithMultiStates) {
    // {0,1} + {0,1} = {0,1,2}
    std::stringstream ss;
    ss <<"+($s(" << stateIndex << ")  $s(" << stateIndex << "))";
    string s = ss.str();
    LogicalExpression* addition = LogicalExpression::createFromString(s);

    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(3, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());

    // 3 + {0,1} + {0,1} = {3,4,5}
    ss.str("");
    ss <<"+($c(3) $s(" << stateIndex << ")  $s(" << stateIndex << "))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);

    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(3, result.size());
    ASSERT_TRUE(result.find(3) != result.end());
    ASSERT_TRUE(result.find(4) != result.end());
    ASSERT_TRUE(result.find(5) != result.end());

    // +($s(1) $s(1) $s(1)) equals to 
    // {0,1} + {0,1} + {0,1} = {0,1,2,3}
    ss.str("");
    ss <<"+($s(" << stateIndex <<") $s(" 
        << stateIndex << ")  $s(" << stateIndex << "))";
    s = ss.str();
    addition = LogicalExpression::createFromString(s);

    result.clear();
    addition->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(4, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(3) != result.end());
}

// Tests kleene evaluation of simple subtractions. Note that subtractions 
// are only defined for two expressions
TEST_F(EvaluateToKleeneTest, testEvalSubtractionWithSingleResult) {
    string s = "-($c(0) $c(1))";
    LogicalExpression* subtraction = LogicalExpression::createFromString(s);
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());

    s = "-($c(1) $c(1))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());


    s = "-($c(0) $c(0))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "-($c(-1) $c(1))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-2) != result.end());

    s = "-($c(-2.5) $c(-2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "-($c(2.5) $c(-2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(5) != result.end());

    s = "-($c(-2.5) $c(2.5))";
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-5) != result.end());
}

// Tests kleene evaluation of subtractions on one multistate
TEST_F(EvaluateToKleeneTest, testEvalSubtractionWithOneMultiState) {
    // {0,1} - 0 = {0,1}
    std::stringstream ss;
    ss <<"-($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* subtraction = LogicalExpression::createFromString(s);

    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // 0 - {0,1} = {0,-1}
    ss.str("");
    ss <<"-($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(-1) != result.end());


    // {0,1} - 1) = {-1,0}
    ss.str("");
    ss <<"-($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // 1 - {0,1) = {1,0}
    ss.str("");
    ss <<"-($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // {0,1} - 2 = {-2,-1}
    ss.str("");
    ss <<"-($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(-2) != result.end());
    ASSERT_TRUE(result.find(-1) != result.end());

    // 2 - {0,1} = {2,1}
    ss.str("");
    ss <<"-($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    subtraction = LogicalExpression::createFromString(s);
    result.clear();
    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of subtractions on multistates
TEST_F(EvaluateToKleeneTest, testEvalSubtractionWithMultiStates) {
    // {0,1} - {0,1} = {0,-1,1}
    std::stringstream ss;
    ss <<"-($s(" << stateIndex << ")  $s(" << stateIndex << "))";
    string s = ss.str();
    LogicalExpression* subtraction = LogicalExpression::createFromString(s);

    subtraction->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(3, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(-1) != result.end());
}

// Tests kleene evaluation of simple multiplications. Note that multiplications 
// are only defined for two expressions
TEST_F(EvaluateToKleeneTest, testEvalMultiplicationWithSingleResult) {
    string s = "*($c(0) $c(1))";
    LogicalExpression* multiplication = LogicalExpression::createFromString(s);
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "*($c(1) $c(1))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());


    s = "*($c(0) $c(0))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "*($c(-1) $c(1))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());

    s = "*($c(-2.5) $c(-2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(6.25) != result.end());

    s = "*($c(2.5) $c(-2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-6.25) != result.end());

    s = "*($c(-2.5) $c(2.5))";
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-6.25) != result.end());
}

// Tests kleene evaluation of multiplications on one multistate
TEST_F(EvaluateToKleeneTest, testEvalMultiplicationWithOneMultiState) {
    // {0,1} * 0 = {0,1}
    std::stringstream ss;
    ss <<"*($s(" << stateIndex << ")  $c(0))";
    string s = ss.str();
    LogicalExpression* multiplication = LogicalExpression::createFromString(s);

    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // 0 * {0,1} = {0,-1}
    ss.str("");
    ss <<"*($c(0) $s(" << stateIndex << "))";
    s = ss.str();
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());


    // {0,1} * 1) = {1,0}
    ss.str("");
    ss <<"*($s(" << stateIndex << ")  $c(1))";
    s = ss.str();
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // 1 * {0,1) = {1,0}
    ss.str("");
    ss <<"*($c(1) $s(" << stateIndex << "))";
    s = ss.str();
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // {0,1} * 2 = {2,0}
    ss.str("");
    ss <<"*($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());

    // 2 * {0,1} = {2,0}
    ss.str("");
    ss <<"*($c(2) $s(" << stateIndex << "))";
    s = ss.str();
    multiplication = LogicalExpression::createFromString(s);
    result.clear();
    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of multiplications on multistates
TEST_F(EvaluateToKleeneTest, testEvalMultiplicationWithMultiStates) {
    // {0,1} * {0,1} = {0,1}
    std::stringstream ss;
    ss <<"*($s(" << stateIndex << ")  $s(" << stateIndex << "))";
    string s = ss.str();
    LogicalExpression* multiplication = LogicalExpression::createFromString(s);

    multiplication->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of simple divisions. Note that divisions 
// are only defined for two expressions
TEST_F(EvaluateToKleeneTest, testEvalDivisionWithSingleResult) {
    string s = "/($c(0) $c(1))";
    LogicalExpression* division = LogicalExpression::createFromString(s);
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    s = "/($c(1) $c(1))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());


    s = "/($c(-1) $c(1))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());

    s = "/($c(-2.5) $c(-2.5))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    s = "/($c(2.5) $c(-2.5))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());

    s = "/($c(-2.5) $c(2.5))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-1) != result.end());

    s = "/($c(-2) $c(5))";
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, dummyState, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(-0.4) != result.end());
}

// Tests kleene evaluation of divisions on one multistate
TEST_F(EvaluateToKleeneTest, testEvalDivisionWithOneMultiState) {
    // {0,1} / 1) = {1,0}
    std::stringstream ss;
    ss <<"/($s(" << stateIndex << ")  $c(1))";
    string s = ss.str();
    LogicalExpression* division = LogicalExpression::createFromString(s);
    division->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());


    // {0,1} / 2 = {2,0}
    ss.str("");
    ss <<"/($s(" << stateIndex << ")  $c(2))";
    s = ss.str();
    division = LogicalExpression::createFromString(s);
    result.clear();
    division->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0.5) != result.end());
    ASSERT_TRUE(result.find(0) != result.end());
}

// Tests kleene evaluation of negation
TEST_F(EvaluateToKleeneTest, testEvalNegation) {
    // ~{0,1} = {0,1}
    std::stringstream ss;
    ss <<"~($s(" << stateIndex << ")  $s(" << stateIndex << "))";
    string s = ss.str();
    LogicalExpression* negation = LogicalExpression::createFromString(s);

    negation->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // ~1 = 0
    ss.str("");
    ss <<"~($c(1))";
    s = ss.str();
    negation = LogicalExpression::createFromString(s);
    result.clear();
    negation->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // ~0 = 1
    ss.str("");
    ss <<"~($c(0))";
    s = ss.str();
    negation = LogicalExpression::createFromString(s);
    result.clear();
    negation->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of bernoulli distribution
TEST_F(EvaluateToKleeneTest, testEvalBernoulli) {
    // Bernoulli(1) = 1
    std::stringstream ss;
    ss <<"Bernoulli($c(1))";
    string s = ss.str();
    LogicalExpression* bernoulli = LogicalExpression::createFromString(s);
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // Bernoulli(0) = 0
    ss.str("");
    ss <<"Bernoulli($c(0))";
    s = ss.str();
    result.clear();
    bernoulli = LogicalExpression::createFromString(s);
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    // Bernoulli(-5) = 1
    ss.str("");
    ss <<"Bernoulli($c(-5))";
    s = ss.str();
    result.clear();
    bernoulli = LogicalExpression::createFromString(s);
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // Bernoulli(5) = 1
    ss.str("");
    ss <<"Bernoulli($c(5))";
    s = ss.str();
    result.clear();
    bernoulli = LogicalExpression::createFromString(s);
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());

    // Bernoulli(0.5) = {0,1}
    ss.str("");
    ss <<"Bernoulli($c(0.5))";
    s = ss.str();
    result.clear();
    bernoulli = LogicalExpression::createFromString(s);
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());

    // Bernoulli( {0,1}) = {0,1}
    ss.str("");
    ss <<"Bernoulli($s(" << stateIndex << "))";
    s = ss.str();
    bernoulli = LogicalExpression::createFromString(s);

    result.clear();
    bernoulli->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0) != result.end());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of discrete distribution
TEST_F(EvaluateToKleeneTest, testEvalDiscrete) {
    // Discrete(1 : 0.4, 2 : 0.4, 3 : 0.2) = {1,2,3}
    std::stringstream ss;
    ss <<"Discrete(($c(1) : $c(0.4)) ($c(2) : $c(0.4)) ($c(3) : $c(0.2)))";
    string s = ss.str();
    LogicalExpression* discrete = LogicalExpression::createFromString(s);
    discrete->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(3, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(3) != result.end());

    // Discrete(1 : 0.4, 2 : 0.4, 3 : 0.2) = {2,3}
    ss.str("");
    ss <<"Discrete(($c(1) : $c(0)) ($c(2) : $c(0.4)) ($c(3) : $c(0.6)))";
    s = ss.str();
    result.clear();
    discrete = LogicalExpression::createFromString(s);
    discrete->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(2) != result.end());
    ASSERT_TRUE(result.find(3) != result.end());

    // Discrete(1 : {0,1}, 2 : 0, 3 : 0) = 1
    ss.str("");
    ss << "Discrete(($c(1) : $s(" << stateIndex << ")) "
        "($c(2) : $c(0)) ($c(3) : $c(0)))";
    s = ss.str();
    result.clear();
    discrete = LogicalExpression::createFromString(s);
    discrete->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(1) != result.end());
}

// Tests kleene evaluation of multiconditions
TEST_F(EvaluateToKleeneTest, testEvalMultiCond) {
    // if (0) then (0.5) else (2) = 2
    std::stringstream ss;
    ss.str("");
    ss << "switch( ($c(0) : $c(0.5)) ($c(1) : $c(2)))";
    string s = ss.str();
    result.clear();
    LogicalExpression* multicond = LogicalExpression::createFromString(s);
    multicond->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(1, result.size());
    ASSERT_TRUE(result.find(2) != result.end());

    // if ({0,1}) then (0.5) else (2) = {0.5, 2}
    ss.str("");
    ss <<"switch( ($s(" << stateIndex << ") : $c(0.5)) ($c(1) : $c(2)))";
    s = ss.str();
    result.clear();
    multicond = LogicalExpression::createFromString(s);
    multicond->evaluateToKleene(result, kleeneT||kleeneF, *actionDummy);
    ASSERT_EQ(2, result.size());
    ASSERT_TRUE(result.find(0.5) != result.end());
    ASSERT_TRUE(result.find(2) != result.end());
}
