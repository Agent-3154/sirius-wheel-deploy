//
// Created by lingwei on 5/20/24.
//
#include "WBC_Locomotion_Ctrl.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"
#include "../tasks/BodyPosTask.h"
#include "../tasks/BodyOriTask.h"
#include "../tasks/LinkPosTask.h"
#include "../tasks/ContactTask.h"

template<typename T>
void LocomotionCtrl<T>::SetupParameter() {
    std::string filename = Config::path_2_config_directory + "config/Control_Parameters.info";
    const std::string setting_name = "WBCTask_Variable";
    std::vector<T> kp_ori, kd_ori, kp_pos, kd_pos, kp_link, kd_link;
    std::vector<T> kp_joint, kd_joint;
    std::vector<int> verbose;
    loadData::loadStdVector(filename, setting_name + ".verbose", verbose, false);

    loadData::loadStdVector(filename, setting_name + ".kp_ori_task", kp_ori, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kd_ori_task", kd_ori, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kp_pos_task", kp_pos, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kd_pos_task", kd_pos, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kp_link_task", kp_link, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kd_link_task", kd_link, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kp_joint", kp_joint, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".kd_joint", kd_joint, verbose[0]);

    for (int i = 0; i < 3; i++) {
        static_cast<BodyPosTask<T> *>(body_pos_task_)->kp_[i] = kp_pos[i];
        static_cast<BodyPosTask<T> *>(body_pos_task_)->kd_[i] = kd_pos[i];
        static_cast<BodyOriTask<T> *>(body_ori_task_)->kp_[i] = kp_ori[i];
        static_cast<BodyOriTask<T> *>(body_ori_task_)->kd_[i] = kd_ori[i];
        for (auto &j: foot_swing_task_) {
            static_cast<LinkPoseTask<T> *>(j)->kp_[i] = kp_link[i];
            static_cast<LinkPoseTask<T> *>(j)->kd_[i] = kd_link[i];
        }
        WBC_Ctrl_Base<T>::kp_joint_[i] = kp_joint[i];
        WBC_Ctrl_Base<T>::kd_joint_[i] = kd_joint[i];
    }
}

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(Quadruped_Base *model): WBC_Ctrl_Base<T>(model), wbc_lcm_(getLcmUrl(255)) {
    // load default model
    body_pos_task_ = new BodyPosTask<T>(WBC_Ctrl_Base<T>::robot_model_);
    body_ori_task_ = new BodyOriTask<T>(WBC_Ctrl_Base<T>::robot_model_);
    for (int i = 0; i < Config::num_legs; i++) {
        foot_contact_[i] = new ContactTask<T>(WBC_Ctrl_Base<T>::robot_model_, i);
        foot_swing_task_[i] = new LinkPoseTask<T>(WBC_Ctrl_Base<T>::robot_model_, i);
    }
    SetupParameter();
}

template<typename T>
void LocomotionCtrl<T>::Clean() {
    WBC_Ctrl_Base<T>::contact_list_.clear();
    WBC_Ctrl_Base<T>::task_list_.clear();
}

// this func test ok.
template<typename T>
void LocomotionCtrl<T>::ContactTaskUpdate(void *input, Control_FSM_Data &data) {
    (void) data;
    input_data_ = static_cast<LocomotionCtrlData<T> *>(input);
    Clean();
    //check ok
    quat_des_ = ori::rpyToQuat(input_data_->pBody_RPY_des);
    // std::cout << "rpy: " << input_data_->pBody_RPY_des.transpose() << std::endl;
    // std::cout << "quat_des_: " << quat_des_.transpose() << std::endl;
    //<< std::endl << "v_ori_des: "
    //              << input_data_->vBody_Ori_des.transpose()
    //              << "\n pbodyDEs: " << input_data_->pBody_des.transpose() << "\n vbody_des: "
    //              << input_data_->vBody_des.transpose()
    //              << "\n abody_des: " <<
    //              input_data_->aBody_des.transpose() << "\n\n";

    print_dim = 0;
    Vec3<T> zero_vec3;
    zero_vec3.setZero();
    body_ori_task_->UpdateTasks(&quat_des_, input_data_->vBody_Ori_des, zero_vec3);
    print_dim += 3;
    body_pos_task_->UpdateTasks(&(input_data_->pBody_des), input_data_->vBody_des, input_data_->aBody_des);
    print_dim += 3;
    WBC_Ctrl_Base<T>::task_list_.push_back(body_ori_task_);
    WBC_Ctrl_Base<T>::task_list_.push_back(body_pos_task_);

    for (int leg(0); leg < 4; ++leg) {
        if (input_data_->contact_state[leg] > 0.) {
            // Contact
            foot_contact_[leg]->setRFDesired((DVec<T>) (input_data_->Fr_des[leg]));
            //            std::cout << "Fr_des id " << leg << " :"<< input_data_->Fr_des[leg].transpose() << "\n";
            foot_contact_[leg]->UpdateTasks();
            WBC_Ctrl_Base<T>::contact_list_.push_back(foot_contact_[leg]);
            print_dim += 3;
        } else {
            // No Contact (swing)
            foot_swing_task_[leg]->UpdateTasks(
                &(input_data_->pFoot_des[leg]),
                input_data_->vFoot_des[leg],
                input_data_->aFoot_des[leg]);
            //zero_vec3);
            WBC_Ctrl_Base<T>::task_list_.push_back(foot_swing_task_[leg]);
        }
    }
}

template<typename T>
void LocomotionCtrl<T>::LCM_PublishData() {
    int iter = 0;
    DVec<double> opt_ori, opt_pos;
    body_ori_task_->getCommand(opt_ori);
    body_pos_task_->getCommand(opt_pos);
    DVec<double> opt_link[4];
    for (int i = 0; i < 4; i++) {
        foot_swing_task_[i]->getCommand(opt_link[i]);
    }

    for (int leg = 0; leg < 4; leg++) {
        Fr_result_[leg].setZero();
        if (input_data_->contact_state[leg] > 0.) {
            for (int i = 0; i < 3; i++) {
                Fr_result_[leg][i] = WBC_Ctrl_Base<T>::wbic_extra_data_->Fr_[3 * iter + i];
            }
            iter++;
        }

        if (input_data_->contact_state[leg] > 0.) {
            wbc_data.contact_esi[leg] = 1;
        } else {
            wbc_data.contact_esi[leg] = 0;
        }
    }

    for (int i = 0; i < 4; i++) {
        Vec3<double> link_pos_ = this->robot_model_->get_pGC(i);
        Vec3<double> link_vel_ = this->robot_model_->get_vGC(i);
        for (int j = 0; j < 3; j++) {
            wbc_data.Fr_des[3 * i + j] = input_data_->Fr_des[i][j];
            wbc_data.Fr[3 * i + j] = Fr_result_[i][j];
            wbc_data.foot_pos_cmd[3 * i + j] = input_data_->pFoot_des[i][j];
            wbc_data.foot_vel_cmd[3 * i + j] = input_data_->vFoot_des[i][j];
            wbc_data.foot_acc_cmd[3 * i + j] = input_data_->aFoot_des[i][j];
            wbc_data.jpos_cmd[3 * i + j] = WBC_Ctrl_Base<T>::des_jpos_[3 * i + j];
            wbc_data.jvel_cmd[3 * i + j] = WBC_Ctrl_Base<T>::des_jvel_[3 * i + j];
            wbc_data.tau_ff[3 * i + j] = WBC_Ctrl_Base<T>::tau_ff_[3 * i + j];
            wbc_data.foot_pos[3 * i + j] = static_cast<float>(link_pos_(j));
            wbc_data.foot_vel[3 * i + j] = static_cast<float>(link_vel_(j));
            wbc_data.link_pos_task_error[i][j] = static_cast<float>(opt_link[i](j));
        }
    }
    Vec3<double> temp_rpy_cmd = ori::quatToRPY(quat_des_);
    Vec3<double> temp_rpy = ori::quatToRPY(WBC_Ctrl_Base<T>::robot_model_->model_ori_);

    for (int i = 0; i < 3; i++) {
        wbc_data.body_pos_cmd[i] = input_data_->pBody_des[i];
        wbc_data.body_vel_cmd[i] = input_data_->vBody_des[i];
        wbc_data.body_ori_cmd[i] = quat_des_[i];
        wbc_data.body_pos[i] = WBC_Ctrl_Base<T>::robot_model_->model_pos_[i];
        wbc_data.body_vel[i] = WBC_Ctrl_Base<T>::robot_model_->model_vel_[i];
        wbc_data.body_ori[i] = WBC_Ctrl_Base<T>::robot_model_->model_ori_[i];
        wbc_data.body_angl_vel[i] = WBC_Ctrl_Base<T>::robot_model_->model_omega_[i];
        wbc_data.body_ang_vel_cmd[i] = input_data_->vBody_Ori_des[i];
        wbc_data.body_rpy[i] = static_cast<float>(temp_rpy(i));
        wbc_data.body_rpy_cmd[i] = static_cast<float>(temp_rpy_cmd(i));
        wbc_data.ori_task_error[i] = static_cast<float>(opt_ori(i));
        wbc_data.pos_task_error[i] = static_cast<float>(opt_pos(i));
    }
    wbc_data.body_ori_cmd[3] = quat_des_[3];
    wbc_data.body_ori[3] = WBC_Ctrl_Base<T>::robot_model_->model_ori_[3];

    for (int i = 0; i < print_dim; i++) {
        wbc_data.opt_result[i] = WBC_Ctrl_Base<T>::wbic_extra_data_->opt_result_[i];
    }
    wbc_lcm_.publish("WBC_CHANNEL", &(wbc_data));
}


template
class LocomotionCtrl<double>;
