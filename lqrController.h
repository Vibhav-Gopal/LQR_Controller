#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>
#include<Eigen/Dense>


class LQRController
{
private:
    Eigen::MatrixXd matA;
    Eigen::MatrixXd matB;
    Eigen::MatrixXd matQ;
    Eigen::MatrixXd matR;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P;
    Eigen::MatrixXd V;
    Eigen::MatrixXd X_target; //TargetState
    Eigen::MatrixXd X_current; //CurrentState
    Eigen::MatrixXd U; //Control Input to system
    Eigen::MatrixXd maxU;
    Eigen::MatrixXd minU;
    int maxIteration = 10000;
    float dt = 0.001;
    double tolerance = 0.000000000001;
public:
    LQRController(Eigen::MatrixXd matrixA, Eigen::MatrixXd matrixB,Eigen::MatrixXd matrixQ,Eigen::MatrixXd matrixR,Eigen::MatrixXd matrixP,Eigen::MatrixXd mV);
    bool solveRiccatiIterationC();
    int dumpVals();
    void setCurrentState(Eigen::MatrixXd state);
    void setTargetState(Eigen::MatrixXd state);
    void calcControlInput();
    void updateState(float dt);
    void setMaxInput(Eigen::MatrixXd max);
    void setMinInput(Eigen::MatrixXd min);
    double getPosition();

};

LQRController::LQRController(Eigen::MatrixXd matrixA, Eigen::MatrixXd matrixB,Eigen::MatrixXd matrixQ,Eigen::MatrixXd matrixR,Eigen::MatrixXd matrixP,Eigen::MatrixXd mV)
{
    matA = matrixA;
    matB = matrixB;
    matQ = matrixQ;
    matR = matrixR;
    P = matrixP;
    V = mV;
}
int LQRController::dumpVals()
{
    std::cout<<"Matrix A is\n"<<matA<<"\n\n";
    std::cout<<"Matrix B is\n"<<matB<<"\n\n";
    std::cout<<"Matrix Q is\n"<<matQ<<"\n\n";
    std::cout<<"Matrix R is\n"<<matR<<"\n\n";
    std::cout<<"Matrix P is\n"<<P<<"\n\n";
    std::cout<<"Optimal Gain K is\n"<<K<<"\n\n";
    std::cout<<"Max Limit\n"<<maxU<<"\n\n";
    std::cout<<"Min Limit\n"<<minU<<"\n\n";
    std::cout<<"Control Input U is\n"<<U<<"\n\n";
    std::cout<<"Current State is\n"<<X_current<<"\n\n";
    std::cout<<"Target State is\n"<<X_target<<"\n\n";
    return 0;
}
bool LQRController::solveRiccatiIterationC() 
{
  auto A = matA;
  auto B = matB;
  auto Q = matQ;
  auto R = matR;
  auto iter_max = maxIteration;
  P = Q; // initialize

  Eigen::MatrixXd P_next;

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd Rinv = R.inverse();

  double diff;
  for (int i = 0; i < iter_max; ++i) {
    P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance) {
      std::cout << "iteration mumber = " << i << std::endl;
      K = Rinv * BT * P;
      return true;
    }
  }
  std::cout<<"Over Iteration Limit\n"<<P<<"\nafter iteration " <<maxIteration <<"\n";
  K = Rinv * BT * P;
  return false; // over iteration limit
}
void LQRController::setCurrentState(Eigen::MatrixXd state){
  X_current = state;
}
void LQRController::setTargetState(Eigen::MatrixXd state){
  X_target = state;
}
void LQRController::calcControlInput(){
  U = -1 * K * (X_current - X_target);
  int rows = U.rows();
  int cols = U.cols();
  for(int i=0;i<rows;i++)
    for(int j=0;j<cols;j++)
    {
      if ((U(i,j)) > (maxU(i,j))) {U = maxU;break;}
      if ((U(i,j)) < (minU(i,j))) {U = maxU;break;}
    }
}
void LQRController::updateState(float dt){
  auto Xdot = (matA * X_current) + (matB * U);
  X_current = (Xdot*dt) + X_current;
}
void LQRController::setMaxInput(Eigen::MatrixXd max){
  maxU = max;
}
void LQRController::setMinInput(Eigen::MatrixXd min){
  minU = min;
}
double LQRController::getPosition(){
  return X_current(0,0);
}