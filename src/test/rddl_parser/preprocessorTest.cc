#include "../gtest/gtest.h"
#include "../../rddl_parser/rddl_parser.h"
#include "../../rddl_parser/instantiator.h"
#include "../../rddl_parser/preprocessor.h"
#include "../../rddl_parser/planning_task.h"
#include "../../rddl_parser/evaluatables.h"
using std::string;
using std::vector;
using std::map;
using std::set;
using std::numeric_limits;

// Skill teaching domain includes +,*,-,neg, state, constant, unaries
TEST(PreprocessorTest, calculateMinMaxRewardWithoutVectorCaching) {
    // Prepare test setting
    string folder = "../test/testdomains/";
    string domainName = "skill_teaching";
    string domainFileName = folder + domainName + "_mdp.rddl_prefix";
    string problemFileName = folder + domainName + "_inst_mdp__9.rddl_prefix";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate(false);
    Preprocessor preprocessor(task);
    preprocessor.preprocess(false);

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR(-19.6723, minValue, 0.0001);
    ASSERT_NEAR(19.6723, maxValue, 0.0001);
}

// Crossing traffic domain includes +,-,neg, state, constant, unaries,
// conjunction
TEST(PreprocessorTest, calculateMinMaxRewardWithoutVectorCachingUnary) {
    // Prepare test setting
    string folder = "../test/testdomains/";
    string domainName = "crossing_traffic";
    string domainFileName = folder + domainName + "_mdp.rddl_prefix";
    string problemFileName = folder + domainName + "_inst_mdp__10.rddl_prefix";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate(false);
    task->rewardCPF->cachingType = "";
    Preprocessor preprocessor(task);
    preprocessor.preprocess(false);

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR(-1, minValue, 0.0001);
    ASSERT_NEAR(0, maxValue, 0.0001);
}

TEST(PreprocessorTest, calculateMinMaxRewardWithoutVectorCachingExists) {
    // Prepare test setting
    string folder = "../test/testdomains/";
    string domainName = "recon";
    string domainFileName = folder + domainName + "_mdp.rddl_prefix";
    string problemFileName = folder + domainName + "_inst_mdp__1.rddl_prefix";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate(false);
    task->rewardCPF->cachingType = "";
    Preprocessor preprocessor(task);
    preprocessor.preprocess(false);

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR( -0.7311116, minValue, 0.0001);
    ASSERT_NEAR(0.18377236, maxValue, 0.0001);
}
