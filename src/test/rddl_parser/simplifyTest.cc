#include "../gtest/gtest.h"
#include "../../rddl_parser/rddl_parser.h"
#include "../../rddl_parser/logical_expressions.h"

using std::string;
using std::vector;
using std::map;

class SimplifyTest : public testing::Test {
protected:
    // Tests accessing private members
    void testNestedIfThenElseSimplification() {
        string folder = "../test/testdomains/";
        string domainName = "test";
        string domainFileName = folder + domainName;
        string problemFileName = folder + domainName + "_inst";

        RDDLParser parser;
        parser.parse(domainFileName, problemFileName);
        string formula = "(if (a) then (if (b) then (c)"
            " else (if (d) then (e) else (f)))"
            " else (if (d) then (b) else (g)))";

        string expected = "switch( (and(a b) : c)" 
            " (and(a d) : e)"
            " (a : f)"
            " (d : b)"
            " ($c(1) : g) )";

        LogicalExpression* expr = parser.parseRDDLFormula(formula);
        map<ParametrizedVariable*, double> replacements;
        expr = expr->simplify(replacements);
        std::stringstream ss;
        expr->print(ss);
        ASSERT_EQ(expected, ss.str());
    }

};

TEST_F(SimplifyTest, testNestedIfThenElseSimplification) {
    testNestedIfThenElseSimplification();
}
