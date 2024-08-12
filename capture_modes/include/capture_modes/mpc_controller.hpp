#pragma once


class MpcController {

public:
    MpcController();
    ~MpcController();

    void init();
    void run();

    float Kp;
    float Kv;
    float Kpz;
    float Kvz;
    float vx2;
    float vy2;
    float vz2;
    float X3;
    float Y3;
    float Z3;
    float vx3;
    float vy3;
    float vz3;
    float dir2;
    float dir3;

    

    std::vector<float> Cp{3};
    std::vector<float> Cv{3};
    
    std::vector<float> acel{3};
    std::vector<float> velocidade{3};

    std::vector<double> xx0;

    std::vector<casadi::DM> arg1;
    std::vector<casadi::DM> res;
    casadi::Matrix<double> result_xx;
    casadi::Matrix<double> result_uu;

    casadi::Matrix<double> result_matrix;

};