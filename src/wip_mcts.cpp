#include "ctp.hpp"
#include "mcts.hpp"

using namespace CTP;
void makeTree() {
  // MCTS::Tree tree;

  // for(int i=0; i<1000000; i++)
  // {
  //     tree.addNode(tree.root);
  // }

  // std::string unused;
  // std::cout << "Waiting for user input\n";
  // std::getline(std::cin, unused);
}

int main() {
  std::mt19937 rng;
  int rows = 10;
  BctpGrid g(rows);

  Agent agent(rows + 1, rows * (rows - 1) - 2);

  CtpProblem<BctpGrid> ctp(g, g.sampleInstance(rng), agent);

  std::string unused;
  std::cout << "Waiting for user input\n";
  std::getline(std::cin, unused);
}
