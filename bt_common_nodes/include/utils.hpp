#pragma once

#include "behaviortree_cpp/bt_factory.h"

/* @brief Allows the client to use BT::getInput with included error checking and value access. */
template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}