#ifndef BLACKBOARD_CHECK_HPP
#define BLACKBOARD_CHECK_HPP

#include <ros/ros.h>

#include "behaviortree_cpp_v3/decorator_node.h"

using namespace BT;

template <class Type>
class BlackBoardCheck : public DecoratorNode
{
public:
  BlackBoardCheck(const std::string & name, const NodeConfiguration& config);

  static PortsList providedPorts();

  NodeStatus tick() override;

  void halt() override;
};

template <typename Type>
BlackBoardCheck<Type>::BlackBoardCheck(const std::string & name, const NodeConfiguration& config):
  DecoratorNode(name, config)
{
}

template <typename Type>
PortsList BlackBoardCheck<Type>::providedPorts()
{
    return { InputPort<Type>("blackboard_entry"), InputPort<Type>("expected"), InputPort<NodeStatus>("return_on_mismatch")};
}

template <typename Type>
NodeStatus BlackBoardCheck<Type>::tick()
{ 

  Type blackboard_entry;
  Type expected;
  NodeStatus return_on_mismatch;

  if( !getInput("blackboard_entry", blackboard_entry) )
  {
      throw RuntimeError("Tree Node: ", name(), " is missing required input [blackboard_entry] ");
  }


  if( !getInput("expected", expected) )
  {
      throw RuntimeError("Tree Node: ", name(), " is missing required input [expected] ");
  }


  if( !getInput("return_on_mismatch", return_on_mismatch) )
  {
      throw RuntimeError("Tree Node: ", name(), " is missing required input [return_on_mismatch] ");
  }

  if (blackboard_entry == expected){
    return child_node_->executeTick();
  }
  
  if( child()->status() == NodeStatus::RUNNING )
  {
      haltChild();
  }

  return return_on_mismatch;
}

template <typename Type>
void BlackBoardCheck<Type>::halt()
{
    DecoratorNode::halt();
}

#endif  
