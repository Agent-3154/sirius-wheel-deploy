//
// Created by lingwei on 5/20/24.
//

#include "ContactTask.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"

template<typename T>
bool ContactTask<T>::UpdateInequalityVector() {
    Contact_Task_Base<T>::ieq_vec_ = DVec<T>::Zero(dim_u_);
    Contact_Task_Base<T>::ieq_vec_[5] = -max_Fz_;
    // Contact_Task_Base<T>::ieq_vec_[0] = max_Fz_;
    return true;
}

/**
 * @note: force here is spatial force
 * @param base_model
 * @param contact_id
 */
template<typename T>
ContactTask<T>::ContactTask(const Quadruped_Base *base_model, int contact_id): Contact_Task_Base<T>(3),
                                                                               dim_u_(6), robot_(base_model),
                                                                               contact_id_(contact_id) {
    Contact_Task_Base<T>::dim_contact_ = 3;

    std::string filename = Config::path_2_config_directory + "config/Control_Parameters.info";
    std::string setting_name = "LinkContactTask_Variable";
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    loadData::loadPtreeValue(pt, max_Fz_, setting_name + ".Max_Fz", false);
    loadData::loadPtreeValue(pt, mu_, setting_name + ".mu", false);

    ContactTask<T>::Jtdqd_ = DVec<T>::Zero(ContactTask<T>::dim_contact_);
    ContactTask<T>::uf_ = DMat<T>::Zero(dim_u_, ContactTask<T>::dim_contact_); // mapping matrix from force to spatial
    ContactTask<T>::uf_(0, 2) = 1.;
    ContactTask<T>::uf_(1, 0) = 1.;
    ContactTask<T>::uf_(1, 2) = mu_;
    ContactTask<T>::uf_(2, 0) = -1.;
    ContactTask<T>::uf_(2, 2) = mu_;

    ContactTask<T>::uf_(3, 1) = 1.;
    ContactTask<T>::uf_(3, 2) = mu_;
    ContactTask<T>::uf_(4, 1) = -1.;
    ContactTask<T>::uf_(4, 2) = mu_;

    // Upper bound of normal force
    ContactTask<T>::uf_(5, 2) = -1.;
}

template<typename T>
bool ContactTask<T>::UpdateTaskJacobian() {
    ContactTask<T>::Jt_ = robot_->get_jc(contact_id_);
    //    std::cout << "Contact Task: " << contact_id_ << "\n" << ContactTask<T>::Jt_ << std::endl;
    return true;
}

template<typename T>
bool ContactTask<T>::UpdateTaskJdqd() {
    ContactTask<T>::Jtdqd_ = robot_->get_jcdqd(contact_id_);
    return true;
}

template<typename T>
bool ContactTask<T>::UpdateUf() {
    return true;
}

template
class ContactTask<double>;
