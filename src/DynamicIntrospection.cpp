#include <dynamic_introspection/DynamicIntrospection.h>
#include <dynamic_introspection/BoolParameter.h>
#include <dynamic_introspection/DoubleParameter.h>
#include <dynamic_introspection/IntParameter.h>
#include <dynamic_introspection/VectorParameter.h>
#include <dynamic_introspection/MatrixParameter.h>
#include <exception>

struct ExistingVariableException : public std::exception
{
  const char * what () const throw ()
  {
    return "Registering an existing variable";
  }
};

struct DoesNotExistingVariableException : public std::exception
{
  const char * what () const throw ()
  {
    return "Trying to delete a variable that is not registered";
  }
};

template<typename C>
bool contains(const std::vector<std::pair<std::string, C*> > & c, const std::string& e)
{
  for(size_t i=0; i<c.size(); ++i){
    if(c[i].first == e){
      return true;
    }
  }
  return false;
}

template <class T>
int indexVector(std::vector<T> v, T e){
  auto it = std::find(v.begin(), v.end(), e);
  if (it == v.end())
  {
    // name not in vector
    return -1;
  } else
  {
    return std::distance(v.begin(), it);
  }
}

DynamicIntrospection* DynamicIntrospection::m_pInstance = NULL;

void copyEigenVector2Message(const Eigen::VectorXd &in, dynamic_introspection::VectorParameter &out){
  assert(in.rows() == out.value.size());
  for(unsigned int i=0; i<in.rows(); ++i){
    out.value[i] = in(i);
  }

}

void copyEigenMatrix2Message(const Eigen::MatrixXd &in, dynamic_introspection::MatrixParameter &out){
  assert(in.rows() == out.rows);
  assert(in.cols() == out.cols);
  assert(in.rows()*in.cols() == out.value.size());

  for(unsigned int i=0; i<in.rows(); ++i){
    for(unsigned int j=0; j<in.cols(); ++j){
      out.value[j + in.rows()*i] = in(i,j);
    }
  }
}


DynamicIntrospection* DynamicIntrospection::Instance(){
  if (!m_pInstance){   // Only allow one instance of class to be generated.
    m_pInstance = new DynamicIntrospection;
  }
  return m_pInstance;
}

/*
DynamicIntrospection::DynamicIntrospection(ros::NodeHandle &nh, const std::string &topic):
  node_handle_(nh), configured_(false), openedBag_(false){
  introspectionPub_ =  nh.advertise<dynamic_introspection::IntrospectionMsg>(topic, 10);
}
*/
DynamicIntrospection::DynamicIntrospection(){
//  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
//    ros::console::notifyLoggerLevelsChanged();
//  }
  node_handle_ = ros::NodeHandle();
  introspectionPub_ =  node_handle_.advertise<dynamic_introspection::IntrospectionMsg>("data", 10);
}

void DynamicIntrospection::setOutputTopic(const std::string &outputTopic){
  introspectionPub_.shutdown();
  introspectionPub_ =  node_handle_.advertise<dynamic_introspection::IntrospectionMsg>(outputTopic, 10);
}

DynamicIntrospection::~DynamicIntrospection(){
  introspectionPub_.shutdown();
  closeBag();
}

void DynamicIntrospection::openBag(std::string fileName){
  bag_.open(fileName, rosbag::bagmode::Write);
  openedBag_ = true;
}

void DynamicIntrospection::closeBag(){
  bag_.close();
  openedBag_ = false;
}

void DynamicIntrospection::publishDataBag(){
  if(!openedBag_){
    ROS_ERROR_STREAM("Bag is not open");
  }
  generateMessage();
  bag_.write("dynamic_introspection", ros::Time::now(), introspectionMessage_);
}

void DynamicIntrospection::publishDataTopic(){
  generateMessage();
  introspectionPub_.publish(introspectionMessage_);
}

void DynamicIntrospection::generateMessage(){

  introspectionMessage_.ints.resize(registeredInt_.size());
  introspectionMessage_.doubles.resize(registeredDouble_.size());
  introspectionMessage_.bools.resize(registeredBool_.size());
  introspectionMessage_.vectors.resize(registeredVector_.size());
  introspectionMessage_.vectors3d.resize(registered3dVector_.size());
  introspectionMessage_.matrixs.resize(registeredMatrix_.size());

  for(unsigned int i=0; i<registeredVector_.size(); ++i){
    introspectionMessage_.vectors[i].value.resize(registeredVector_[i].second->rows());
  }

  for(unsigned int i=0; i<registeredMatrix_.size(); ++i){
    introspectionMessage_.matrixs[i].value.resize(registeredMatrix_[i].second->rows()*registeredMatrix_[i].second->cols());
  }

  for(unsigned int i=0; i<registeredInt_.size(); ++i){
    dynamic_introspection::IntParameter &ip = introspectionMessage_.ints[i];
    ip.name = registeredInt_[i].first;
    ip.value = *registeredInt_[i].second;
  }

  for(unsigned int i=0; i<registeredDouble_.size(); ++i){
    dynamic_introspection::DoubleParameter &dp = introspectionMessage_.doubles[i];
    dp.name = registeredDouble_[i].first;
    dp.value = *registeredDouble_[i].second;
  }

  for(unsigned int i=0; i<registeredBool_.size(); ++i){
    dynamic_introspection::BoolParameter &bp = introspectionMessage_.bools[i];
    bp.name = registeredBool_[i].first;
    bp.value = *registeredBool_[i].second;
  }

  for(unsigned int i=0; i<registered3dVector_.size(); ++i){
    dynamic_introspection::VectorParameter &vp = introspectionMessage_.vectors3d[i];
    vp.name = registered3dVector_[i].first;
    assert(registered3dVector_[i].second->rows() == 3);
    vp.value.resize(registered3dVector_[i].second->rows());
    copyEigenVector2Message(*registered3dVector_[i].second, vp);
  }

  for(unsigned int i=0; i<registeredVector_.size(); ++i){
    dynamic_introspection::VectorParameter &vp = introspectionMessage_.vectors[i];
    vp.name = registeredVector_[i].first;
    vp.value.resize(registeredVector_[i].second->rows());
    copyEigenVector2Message(*registeredVector_[i].second, vp);
  }

  for(unsigned int i=0; i<registeredMatrix_.size(); ++i){
    dynamic_introspection::MatrixParameter &mp = introspectionMessage_.matrixs[i];
    mp.name = registeredMatrix_[i].first;
    mp.rows = registeredMatrix_[i].second->rows();
    mp.cols =  registeredMatrix_[i].second->cols();
    mp.value.resize(mp.rows*mp.cols);
    copyEigenMatrix2Message(*registeredMatrix_[i].second, mp);
  }
}

void DynamicIntrospection::registerVariable(int *variable, std::string id){
  if(contains(registeredInt_, id)){
    ROS_ERROR_STREAM("Int : "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
   ROS_DEBUG_STREAM("Registered int: "<<id);
   std::pair<std::string, int*> p(id, variable);
   registeredInt_.push_back(p);
  }
}

void DynamicIntrospection::registerVariable(double *variable, std::string id){
  if(contains(registeredDouble_, id)){
    ROS_ERROR_STREAM("Double : "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
   ROS_DEBUG_STREAM("Registered double: "<<id);
   std::pair<std::string, double*> p(id, variable);
   registeredDouble_.push_back(p);
  }
}

void DynamicIntrospection::registerVariable(bool *variable, std::string id){
  if(contains(registeredBool_, id)){
    ROS_ERROR_STREAM("Bool: "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
    ROS_DEBUG_STREAM("Registered bool: "<<id);
    std::pair<std::string, bool*> p(id, variable);
    registeredBool_.push_back(p);
  }
}

void DynamicIntrospection::registerVariable(Eigen::Vector3d *variable, std::string id){
  if(contains(registered3dVector_, id)){
    ROS_ERROR_STREAM("Vector3: "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
    ROS_DEBUG_STREAM("Registered Vector3: "<<id);
    std::pair<std::string, Eigen::Vector3d*> p(id, variable);
    registered3dVector_.push_back(p);
  }
}

void DynamicIntrospection::registerVariable(Eigen::VectorXd *variable, std::string id){
  if(contains(registeredVector_, id)){
    ROS_ERROR_STREAM("Vector: "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
    ROS_DEBUG_STREAM("Registered Vector: "<<id);
    std::pair<std::string, Eigen::VectorXd*> p(id, variable);
    registeredVector_.push_back(p);
  }
}

void DynamicIntrospection::registerVariable(Eigen::MatrixXd *variable, std::string id){
  if(contains(registeredMatrix_, id)){
    ROS_ERROR_STREAM("Matrix: "<<id<<" has allreday been registered");
    throw ExistingVariableException();
  }
  else{
    ROS_DEBUG_STREAM("Registered Matrix: "<<id);
    std::pair<std::string, Eigen::MatrixXd*> p(id, variable);
    registeredMatrix_.push_back(p);
  }
}

/////////

void DynamicIntrospection::unRegisterVariable(int *variable, std::string id){
  if(!contains(registeredInt_, id)){
    ROS_ERROR_STREAM("Int : "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
   std::pair<std::string, int*> p(id, variable);
   int index = indexVector(registeredInt_, p);
   if(index < 0){
     ROS_ERROR_STREAM("Int : "<<id<<" has has been registered but the pointer to its data does not match!");
   }
   else{
     ROS_DEBUG_STREAM("Deleting int: "<<id);
     registeredInt_.erase(registeredInt_.begin() + index);
   }
  }
}

void DynamicIntrospection::unRegisterVariable(double *variable, std::string id){
  if(!contains(registeredDouble_, id)){
    ROS_ERROR_STREAM("Double : "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
    std::pair<std::string, double*> p(id, variable);
    int index = indexVector(registeredDouble_, p);
    if(index < 0){
      ROS_ERROR_STREAM("Double : "<<id<<" has has been registered but the pointer to its data does not match!");
    }
    else{
      ROS_DEBUG_STREAM("Deleting int: "<<id);
      registeredDouble_.erase(registeredDouble_.begin() + index);
    }
  }
}

void DynamicIntrospection::unRegisterVariable(bool *variable, std::string id){
  if(!contains(registeredBool_, id)){
    ROS_ERROR_STREAM("Bool: "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
    std::pair<std::string, bool*> p(id, variable);
    int index = indexVector(registeredBool_, p);
    if(index < 0){
      ROS_ERROR_STREAM("Bool : "<<id<<" has has been registered but the pointer to its data does not match!");
    }
    else{
      ROS_DEBUG_STREAM("Deleting int: "<<id);
      registeredBool_.erase(registeredBool_.begin() + index);
    }
  }
}

void DynamicIntrospection::unRegisterVariable(Eigen::Vector3d *variable, std::string id){
  if(!contains(registered3dVector_, id)){
    ROS_ERROR_STREAM("Vector3: "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
    std::pair<std::string, Eigen::Vector3d*> p(id, variable);
    int index = indexVector(registered3dVector_, p);
    if(index < 0){
      ROS_ERROR_STREAM("Vector3 : "<<id<<" has has been registered but the pointer to its data does not match!");
    }
    else{
      ROS_DEBUG_STREAM("Deleting int: "<<id);
      registered3dVector_.erase(registered3dVector_.begin() + index);
    }
  }
}

void DynamicIntrospection::unRegisterVariable(Eigen::VectorXd *variable, std::string id){
  if(!contains(registeredVector_, id)){
    ROS_ERROR_STREAM("Vector: "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
    std::pair<std::string, Eigen::VectorXd*> p(id, variable);
    int index = indexVector(registeredVector_, p);
    if(index < 0){
      ROS_ERROR_STREAM("Vector : "<<id<<" has has been registered but the pointer to its data does not match!");
    }
    else{
      ROS_DEBUG_STREAM("Deleting int: "<<id);
      registeredVector_.erase(registeredVector_.begin() + index);
    }
  }
}

void DynamicIntrospection::unRegisterVariable(Eigen::MatrixXd *variable, std::string id){
  if(!contains(registeredMatrix_, id)){
    ROS_ERROR_STREAM("Matrix: "<<id<<" has NOT been registered");
    throw DoesNotExistingVariableException();
  }
  else{
    std::pair<std::string, Eigen::MatrixXd*> p(id, variable);
    int index = indexVector(registeredMatrix_, p);
    if(index < 0){
      ROS_ERROR_STREAM("Matrix : "<<id<<" has has been registered but the pointer to its data does not match!");
    }
    else{
      ROS_DEBUG_STREAM("Deleting int: "<<id);
      registeredMatrix_.erase(registeredMatrix_.begin() + index);
    }
  }
}

