#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include "as2_core/node.hpp"

namespace controller_base {
class ControllerBase{
  public:
  ControllerBase(){};
  virtual void initialize(as2::Node *node_ptr){
    node_ptr_ = node_ptr;
  };

  virtual void updateState()=0;
  virtual void updateReference()=0;
  virtual void computeOutput()=0;
  virtual bool setMode()=0;

  };
  virtual ~ControllerBase(){};
  // To initialize needed publisher for each plugin
  private:
  protected:
  as2::Node *node_ptr_;
};  // ControllerBase class

}  // namespace controller_base

#endif  // CONTROLLER_BASE_HPP
