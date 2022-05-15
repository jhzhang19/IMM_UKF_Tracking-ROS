/*
 * author: wx
 * date: 2020.12.08
 * reference:https://github.com/zhujun98/sensor-fusion
 */
#ifndef MY_UKF_H
#define MY_UKF_H
#include <iostream>
#include <stdlib.h>
#include <string>
#include <Eigen/Geometry>
#include "Eigen/Dense"
#include <unordered_map>

class UKF
{
public:
  UKF(std::string smodel, int state_n, int mea_n, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P)
  {
    //通过模型名称来获取相应的模型编号
    if (model_hash_.count(smodel))
      model_ = model_hash_[smodel];

    n_z_ = mea_n;   //测量量维度
    n_x_ = state_n; //状态向量维度

    //状态向量
    x_ = Eigen::VectorXd(n_x_);
    x_.fill(0.0);

    //协方差矩阵
    P_ = Eigen::MatrixXd(n_x_, n_x_);
    P_ = P;
    //协方差噪声矩阵
    Q_ = Eigen::MatrixXd(n_x_, n_x_);
    Q_ = Q;
    //观测噪声矩阵
    R_ = Eigen::MatrixXd(n_z_, n_z_);
    R_ = R;

    S_ = Eigen::MatrixXd(n_z_, n_z_);
    S_.fill(0.0);

    //观测向量
    Zminus_ = Eigen::VectorXd(n_z_);
    Zminus_.fill(0.0);
  };

  ~UKF(){};

  void Initialization(Eigen::VectorXd &X, Eigen::MatrixXd &P, float time);

  bool Isinitalized();

  void MakeSigmaPoints();

  void Prediction(float ntime);

  void PredictionZ(Eigen::VectorXd &X, Eigen::MatrixXd &P, float ntime);

  void Update(std::vector<Eigen::VectorXd> &Z, const Eigen::VectorXd &beta, const float &last_beta);

  void Update(Eigen::VectorXd &Z);

  void Process(Eigen::VectorXd &X, std::vector<Eigen::VectorXd> &Z, const Eigen::VectorXd &beta, const float &last_beta, Eigen::MatrixXd &P, float time);

  Eigen::VectorXd Get_state();

  Eigen::MatrixXd Get_covariance();

  Eigen::VectorXd Get_Zminus();

  Eigen::MatrixXd Get_S();

  Eigen::VectorXd Get_PredictionZ();

private:
  bool is_initalized_ = false;
  int n_z_; //量测维度
  int n_aug_;
  int n_x_; //状态维度

  float lamda_;
  float pretime;

  bool weight_make_ = false;

  std::unordered_map<std::string, int> model_hash_ = {{"CV", 1}, {"CTRV", 2}, {"CTRA", 3}}; //模型种类字典
  int model_ = 1;                                                                           //模型类别编号

  Eigen::VectorXd x_; //state vector
  Eigen::MatrixXd P_; //状态协方差

  Eigen::MatrixXd Q_; //state白噪声
  Eigen::MatrixXd R_; //measure白噪声

  Eigen::VectorXd pre_weight; //ukf粒子点的权重
  Eigen::VectorXd mea_weight; //测量阶段的权重

  Eigen::MatrixXd sigma_points;     //sigma点
  Eigen::MatrixXd sigma_points_pre; //预测sigma

  Eigen::MatrixXd Z_sigma_;

  Eigen::MatrixXd S_;
  Eigen::VectorXd Zminus_;
  Eigen::VectorXd z_pre_;
};
#endif
