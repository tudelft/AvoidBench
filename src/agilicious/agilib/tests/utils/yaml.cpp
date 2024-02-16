#include "agilib/utils/yaml.hpp"

#include <gtest/gtest.h>

#include <iostream>

using namespace agi;

TEST(Yaml, Parsing) {
  std::string s{
    "int: 2   # a comment\n"
    "node:    # another comment \n"
    "  int: 1\n"
    "  float: 1.0\n"
    "  # an indented comment\n"
    "  double: 2.0\n"
    "  bool: true\n"
    "  string1: \"string1\"\n"
    "  string2: \'string2\'\n"
    "	string3:	string3  # this line contains tabs instead of spaces\n"
    "\tstring4:\t\'string4\" # this line contains escaped tabs\n"
    "  string5: \"string5\'\n"
    "# completely stupid comment followed by empty line\n"
    "\n"
    "  list: [0, 1, 2]\n"
    "  multi_list: [ [1 ,2], [1.0, -2.0, 3.0], [[\"a\", \"b\"], [\'cd\', "
    "\'efg\"]]]\n"
    "  other_list: [{braces}, (parentheses), [mixed}]\n"
    "  complex: (1.0, 2.0)\n"
    "  nested:\n"
    "    nested_value: 3.0\n"
    "    nested_list:\n"
    "      - bla: 1\n"
    "        blu: 3\n"
    "      - bla: 2\n"
    "        blu: 4\n"};

  Yaml yaml(s);
  EXPECT_FALSE(yaml["doesnotexist"].isValid());
  EXPECT_FALSE(yaml["doesnotexist"].isDefined());
  EXPECT_EQ(yaml["doesnotexist"].size(), 0);
  EXPECT_EQ(yaml["int"].as<int>(), 2);
  EXPECT_EQ(yaml["int"].size(), 1ul);
  EXPECT_EQ(yaml["node"].size(), 14);
  EXPECT_EQ(yaml["node"]["int"].as<int>(), 1);
  EXPECT_EQ(yaml["node"]["float"].as<float>(), 1.0f);
  EXPECT_EQ(yaml["node"]["double"].as<double>(), 2.0);
  EXPECT_EQ(yaml["node"]["bool"].as<bool>(), true);
  EXPECT_EQ(yaml["node"]["string1"].as<std::string>(), "string1");
  EXPECT_EQ(yaml["node"]["string2"].as<std::string>(), "string2");
  EXPECT_EQ(yaml["node"]["string3"].as<std::string>(), "string3");
  EXPECT_EQ(yaml["node"]["string4"].as<std::string>(), "string4");
  EXPECT_EQ(yaml["node"]["string5"].as<std::string>(), "string5");
  EXPECT_EQ(yaml["node"]["list"].size(), 3);
  EXPECT_EQ(yaml["node"]["list"][0].as<int>(), 0);
  EXPECT_EQ(yaml["node"]["list"][1].as<int>(), 1);
  EXPECT_EQ(yaml["node"]["list"][2].as<int>(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"].size(), 3);
  EXPECT_EQ(yaml["node"]["multi_list"][0].size(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"][0][0].as<int>(), 1);
  EXPECT_EQ(yaml["node"]["multi_list"][0][1].as<int>(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"][1].size(), 3);
  EXPECT_EQ(yaml["node"]["multi_list"][1][0].as<int>(), 1);
  EXPECT_EQ(yaml["node"]["multi_list"][1][1].as<int>(), -2);
  EXPECT_EQ(yaml["node"]["multi_list"][1][2].as<int>(), 3);
  EXPECT_EQ(yaml["node"]["multi_list"][2].size(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"][2][0].size(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"][2][0][0].as<std::string>(), "a");
  EXPECT_EQ(yaml["node"]["multi_list"][2][0][1].as<std::string>(), "b");
  EXPECT_EQ(yaml["node"]["multi_list"][2][1].size(), 2);
  EXPECT_EQ(yaml["node"]["multi_list"][2][1][0].as<std::string>(), "cd");
  EXPECT_EQ(yaml["node"]["multi_list"][2][1][1].as<std::string>(), "efg");
  EXPECT_EQ(yaml["node"]["other_list"].size(), 3);
  EXPECT_EQ(yaml["node"]["other_list"][0].size(), 1);
  EXPECT_EQ(yaml["node"]["other_list"][1].size(), 1);
  EXPECT_EQ(yaml["node"]["other_list"][2].size(), 1);
  EXPECT_EQ(yaml["node"]["other_list"][0].as<std::string>(), "braces");
  EXPECT_EQ(yaml["node"]["other_list"][1].as<std::string>(), "parentheses");
  EXPECT_EQ(yaml["node"]["other_list"][2].as<std::string>(), "mixed");
  EXPECT_EQ(yaml["node"]["complex"].size(), 2);
  EXPECT_EQ(yaml["node"]["complex"].as<std::complex<double>>(),
            std::complex<double>(1.0, 2.0));
  EXPECT_EQ(yaml["node"]["nested"]["nested_value"].as<double>(), 3.0);
  EXPECT_EQ(yaml["node"]["nested"]["nested_list"].size(), 2);
  EXPECT_EQ(yaml["node"]["nested"]["nested_list"][0]["bla"].as<int>(), 1);
  EXPECT_EQ(yaml["node"]["nested"]["nested_list"][0]["blu"].as<int>(), 3);
  EXPECT_EQ(yaml["node"]["nested"]["nested_list"][1]["bla"].as<int>(), 2);
  EXPECT_EQ(yaml["node"]["nested"]["nested_list"][1]["blu"].as<int>(), 4);
}


TEST(Yaml, Errors) {
  const std::string ill_formed1{
    "fine_int: 5\n"
    "good_string: \"soo good\"\n"
    " this is not so good .... ::: {}\n"
    "this_could_be_fine_again: 42"};
  EXPECT_ANY_THROW(Yaml yaml(ill_formed1););

  const std::string ill_formed2{
    "fine_int: 5\n"
    "good_string: \"soo good\"\n"
    "too_many_opening_brackets_fail: {{1.0, 2.0}\n"
    "this_could_be_fine_again: 42"};
  EXPECT_ANY_THROW(Yaml yaml(ill_formed2););

  const std::string ill_formed3{
    "fine_int: 5\n"
    "good_string: \"soo good\"\n"
    "too_many_closing_brackets_fail: {1.0, 2.0}}\n"
    "this_could_be_fine_again: 42"};
  EXPECT_ANY_THROW(Yaml yaml(ill_formed3););
}

TEST(Yaml, Bools) {
  const std::string bools{
    "true_1: 1\n"
    "false_0: 0\n"
    "true_t: t\n"
    "false_f: f\n"
    "true_T: T\n"
    "false_F: F\n"
    "true: true\n"
    "false: false\n"
    "true_case: TruE\n"
    "false_case: fALse\n"
    "true_quote: \"true\"\n"
    "false_quote: \"false\"\n"
    "true_sentence: \"if it has true and false, the first is returned\"\n"};
  Yaml yaml{bools};
  std::cout << yaml;
  EXPECT_TRUE(yaml["true_1"].as<bool>());
  EXPECT_FALSE(yaml["false_0"].as<bool>());
  EXPECT_TRUE(yaml["true_t"].as<bool>());
  EXPECT_FALSE(yaml["false_f"].as<bool>());
  EXPECT_TRUE(yaml["true_T"].as<bool>());
  EXPECT_FALSE(yaml["false_F"].as<bool>());
  EXPECT_TRUE(yaml["true"].as<bool>());
  EXPECT_FALSE(yaml["false"].as<bool>());
  EXPECT_TRUE(yaml["true_case"].as<bool>());
  EXPECT_FALSE(yaml["false_case"].as<bool>());
  EXPECT_TRUE(yaml["true_quote"].as<bool>());
  EXPECT_FALSE(yaml["false_quote"].as<bool>());
  EXPECT_TRUE(yaml["true_sentence"].as<bool>());
}
