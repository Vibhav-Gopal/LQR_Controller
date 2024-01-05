#include "lqrController.h"
#define MASS 16.0
#define DRAG_COEFF 0.5

#define m MASS
#define c DRAG_COEFF
int main()
{
    //Define A B matrices
    Eigen::MatrixXd matA(2,2);
    matA(0,0) = 0;
    matA(0,1) = 1;
    matA(1,0) = 0;
    matA(1,1) = -c/m;

    Eigen::MatrixXd matB(2,1);
    matB(0,0) = 0;
    matB(1,0) = 1.000/m;

    //Define matrix Q
    Eigen::MatrixXd matQ(2,2);
    matQ(0,0) = 20;
    matQ(0,1) = 0;
    matQ(1,0) = 0;
    matQ(1,1) = 10;

    //Define matrix R
    Eigen::MatrixXd matR(1,1);
    matR(0,0) = 0.1;
    
    //Define one more matrix V bcoz LQR doesnt give a flying shit about steady state errors
    Eigen::MatrixXd matV(2,2);
    matV(0,0) = 1;
    matV(0,1) = 0;
    matV(1,0) = 0;
    matV(1,1) = 1;

    Eigen::MatrixXd curr(2,1);
    curr(0,0) = 3;
    curr(1,0) = 0;

    Eigen::MatrixXd target(2,1);
    target(0,0) = 0;
    target(1,0) = 0;

    Eigen::MatrixXd maxU(1,1);
    maxU(0,0) = 75;
    Eigen::MatrixXd minU(1,1);
    minU(0,0) = -75;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);

    auto myLQr = LQRController(matA,matB,matQ,matR,P,matV);
    bool status = myLQr.solveRiccatiIterationC();
    myLQr.setCurrentState(curr);
    myLQr.setTargetState(target);
    myLQr.setMaxInput(maxU);
    myLQr.setMinInput(minU);
    myLQr.calcControlInput();
    myLQr.dumpVals();
    float dt =0.001;
    float seconds = 10;
    double cycles = (double)seconds/dt;
    std::ofstream file("lqr_results.txt");
    file<<"dt="<<dt<<"\n";
    file<<"targetpos="<<target(0,0)<<"\n";
    file<<"currentpos="<<curr(0,0)<<"\n";
    std::cout<<"Simulating for "<< seconds<<" seconds\n";
    int i;
    for (i=0; i<cycles;i++){
      file<<i<<" "<<myLQr.getPosition()<<"\n";
      myLQr.calcControlInput();
      myLQr.updateState(dt);
    }
    file<<i<<" "<<myLQr.getPosition();
    file.close();
    myLQr.dumpVals();
    return 0;
}
