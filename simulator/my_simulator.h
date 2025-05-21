#ifndef MUJOCO_MY_SIMULATOR_H_
#define MUJOCO_MY_SIMULATOR_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>

#include "../config/Config.h"
#include "include/platform_ui_adapter.h"
#include "lcm/lcm-cpp.hpp"
#include "../utilities/types/hardware_types.h"
#include "../lcm-types/cpp/imu_lcmt.hpp"
#include "../lcm-types/cpp/usb_data_t.hpp"

namespace mujoco {
    typedef struct user_geoms {
        mjvGeom geom_foot_d_[4][Config::drawing_geom_number_feets];
        mjtNum from_foot_d_[4][3 * Config::drawing_geom_number_feets];
        mjtNum to_foot_d_[4][3 * Config::drawing_geom_number_feets];
        size_t active_geoms_des_[4];

        mjvGeom geom_foot_[4][Config::drawing_geom_number_feets];
        mjtNum from_foot_[4][3 * Config::drawing_geom_number_feets];
        mjtNum to_foot_[4][3 * Config::drawing_geom_number_feets];
        size_t active_geoms_[4];

        mjvGeom pos_d_[Config::drawing_geom_number_pw];
        mjtNum from_pos_d_[3 * Config::drawing_geom_number_pw];
        mjtNum to_pos_d_[3 * Config::drawing_geom_number_pw];
        size_t active_geoms_pos_des_;

        mjvGeom pos_[Config::drawing_geom_number_pw];
        mjtNum from_pos_[3 * Config::drawing_geom_number_pw];
        mjtNum to_pos_[3 * Config::drawing_geom_number_pw];
        size_t active_geoms_pos_;

    } user_geoms_t;

    // recursive_mutex在同一个线程能够锁多次
    class SimulateMutex : public std::recursive_mutex {
    };

    using MutexLock = std::unique_lock<std::recursive_mutex>;

    class Simulate {
    public:
        using Clock = std::chrono::steady_clock;
        static_assert(std::ratio_less_equal_v<Clock::period, std::milli>);
        static constexpr int kMaxGeom = 200000;

        Simulate(std::unique_ptr<PlatformUIAdapter> platfor_ui_adpator, bool is_passive, bool real_sensor,
                 bool usb2can);

        // Synchronize mjModel and mjData state with UI input
        void Sync();

        void LoadMessage(std::string &displayed_filename);

        // Request that the simulate UI thread render a new model
        void Load(mjModel *m, mjData *d, std::string &displayed_filename);

        void LoadMessageClear();

        // Load mjb or xml model that has been requested by load();
        void LoadOnRenderThread();

        void Render();

        void RenderLoop();

        void AddToHistory();

        void set_foot_trajectory_des(const mjtNum from[3], const mjtNum to[3], int leg_id);

        void set_foot_trajectory(const mjtNum from[3], const mjtNum to[3], int leg_id);

        void set_pos_trajectory_des(const mjtNum from[3], const mjtNum to[3]);

        void set_pos_trajectory(const mjtNum from[3], const mjtNum to[3]);

        void set_mpc_force(const mjtNum fr[3], int leg_id);

        void draw_user_geoms();

        void set_draw_traj(const bool draw) { draw_traj_atomic_.store(draw); }

        static constexpr int kMaxFileNameLength = 1000;
        bool is_passive_ = false; //TODO what's this?
        bool is_real_sensor_ = false;
        mjModel *mnew_ = nullptr;
        mjData *dnew_ = nullptr;
        mjModel *m_ = nullptr;
        mjData *d_ = nullptr;

        imu_lcmt sim_local_imudata;
        std::mutex sim_imu_mtx_;

        int ncam_ = 0;
        int nkey_ = 0;
        int state_size_ = 0; // number of mjtNums in a history buffer state
        int nhistory_ = 0; // number of states saved in history buffer
        int history_cursor_ = 0; // cursor pointing at last saved state

        // joint and actuators
        std::vector<int> body_parentid_;
        std::vector<int> jnt_type_;
        std::vector<int> jnt_group_;
        std::vector<int> jnt_qposadr_;
        std::vector<std::optional<std::pair<mjtNum, mjtNum> > > jnt_range_;
        std::vector<std::string> jnt_names_;

        std::vector<int> actuator_group_;
        std::vector<std::optional<std::pair<mjtNum, mjtNum> > > actuator_ctrlrange_;
        std::vector<std::string> actuator_names_;

        std::vector<mjtNum> history_; // history buffer (nhistory x state_size)

        // mjModel and mjData fields that can be modified by the user through the GUI
        std::vector<mjtNum> qpos_;
        std::vector<mjtNum> qpos_prev_;
        std::vector<mjtNum> ctrl_;
        std::vector<mjtNum> ctrl_prev_;

        mjvSceneState scnstate_;
        mjOption mjopt_prev_;
        mjvOption opt_prev_;
        mjvCamera cam_prev_;

        int warn_vgeomfull_prev_;

        // pending GUI-driven actions, to be applied at the next call to Sync
        struct {
            std::optional<std::string> save_xml;
            std::optional<std::string> save_mjb;
            std::optional<std::string> print_model;
            std::optional<std::string> print_data;
            bool print_dynamic;
            bool reset;
            bool align;
            bool copy_pose;
            bool load_from_history;
            bool load_key;
            bool save_key;
            bool zero_ctrl;
            int newperturb;
            bool select;
            mjuiState select_state;
            bool ui_update_simulation;
            bool ui_update_physics;
            bool ui_update_rendering;
            bool ui_update_joint;
            bool ui_update_ctrl;
            bool ui_remake_ctrl;
        } pending_ = {};

        SimulateMutex mtx;
        std::condition_variable_any cond_loadrequest;

        int frames_ = 0;
        std::chrono::time_point<Clock> last_fps_update_;
        double fps_ = 0;

        // options
        int spacing = 0;
        int color = 0;
        int font = 0;
        int ui0_enable = 1;
        int ui1_enable = 1;
        int help = 0;
        int info = 0;
        int profiler = 0;
        int sensor = 0;
        int real_sensor = 0;
        int use_usb2can_ = 0;
        int pause_update = 1;
        int fullscreen = 0;
        int vsync = 1;
        int busywait = 0;

        int lcm_pub_ = 0;
        int draw_traject = 0;

        // keyframe index
        int key = 0;

        // index of history-scrubber slider
        int scrub_index = 0;

        // simulation
        int run = 1;

        // atomics for cross-thread messages
        std::atomic_int exitrequest = 0;
        std::atomic_int droploadrequest = 0;
        std::atomic_int screenshotrequest = 0;
        std::atomic_int uiloadrequest = 0;

        // loadrequest
        //   3: display a loading message
        //   2: render thread asked to update its model
        //   1: showing "loading" label, about to load
        //   0: model loaded or no load requested.
        int loadrequest = 0;

        // strings
        char load_error[kMaxFileNameLength] = "";
        char dropfilename[kMaxFileNameLength] = "";
        std::string filename;
        std::string previous_filename;

        // time synchronization
        int real_time_index = 0;
        bool speed_changed = true;
        float measured_slowdown = 1.0;
        // logarithmically spaced real-time slow-down coefficients (percent)
        static constexpr float percentRealTime[] = {
            100, 80, 66, 50, 40, 33, 25, 20, 16, 13,
            10, 8, 6.6, 5.0, 4, 3.3, 2.5, 2, 1.6, 1.3,
            1, .8, .66, .5, .4, .33, .25, .2, .16, .13,
            .1
        };

        // control noise
        double ctrl_noise_std = 0.0;
        double ctrl_noise_rate = 0.0;

        // watch
        char field[mjMAXUITEXT] = "qpos";
        int index = 0;

        // physics: need sync
        int disable[mjNDISABLE] = {0};
        int enable[mjNENABLE] = {0};
        int enableactuator[mjNGROUP] = {0};

        // rendering: need sync
        int camera = 0;

        // abstract visualization
        mjvScene scn = {};
        mjvCamera cam = {}; //visual option
        mjvOption opt = {}; // solver option
        mjvPerturb pert = {};
        mjvFigure figconstraint = {};
        mjvFigure figcost = {};
        mjvFigure figtimer = {};
        mjvFigure figsize = {};
        mjvFigure figsensor_bodygyro = {};
        mjvFigure figsensor_bodyquat = {};
        mjvFigure figsensor_bodyacc = {};

        // additional user-defined visualization geoms (used in passive mode)
        mjvScene *user_scn = nullptr;
        mjtByte user_scn_flags_prev_[mjNRNDFLAG];

        // OpenGL rendering and UI
        int refresh_rate = 60;
        int window_pos[2] = {0};
        int window_size[2] = {0};
        std::unique_ptr<PlatformUIAdapter> platform_ui;
        mjuiState &uistate;
        mjUI ui0 = {};
        mjUI ui1 = {};

        // Constant arrays needed for the option section of UI and the UI interface
        // TODO setting the size here is not ideal
        mjuiDef def_option[13] = {};
        mjuiDef def_usr[4] = {};
        // simulation section of UI
        const mjuiDef def_simulation[14] = {
            {mjITEM_SECTION, "Simulation", 1, nullptr, "AS"},
            {mjITEM_RADIO, "", 5, &this->run, "Pause\nRun"},
            {mjITEM_BUTTON, "Reset", 2, nullptr, " #259"},
            {mjITEM_BUTTON, "Reload", 5, nullptr, "CL"},
            {mjITEM_BUTTON, "Align", 2, nullptr, "CA"},
            {mjITEM_BUTTON, "Copy pose", 2, nullptr, "CC"},
            {mjITEM_SLIDERINT, "Key", 3, &this->key, "0 0"},
            {mjITEM_BUTTON, "Load key", 3},
            {mjITEM_BUTTON, "Save key", 3},
            {mjITEM_SLIDERNUM, "Noise scale", 5, &this->ctrl_noise_std, "0 2"},
            {mjITEM_SLIDERNUM, "Noise rate", 5, &this->ctrl_noise_rate, "0 2"},
            {mjITEM_SEPARATOR, "History", 1},
            {mjITEM_SLIDERINT, "", 5, &this->scrub_index, "0 0"},
            {mjITEM_END}
        };

        // watch section of UI
        const mjuiDef def_watch[5] = {
            {mjITEM_SECTION, "Watch", 0, nullptr, "AW"},
            {mjITEM_EDITTXT, "Field", 2, this->field, "qpos"},
            {mjITEM_EDITINT, "Index", 2, &this->index, "1"},
            {mjITEM_STATIC, "Value", 2, nullptr, " "},
            {mjITEM_END}
        };

        // info strings
        char info_title[Simulate::kMaxFileNameLength] = {0};
        char info_content[Simulate::kMaxFileNameLength] = {0};

        // pending uploads
        std::condition_variable_any cond_upload_;

        std::atomic_bool draw_traj_atomic_{};
        user_geoms_t user_geoms{};
        int geom_iter_des_[4]{};
        int geom_iter_[4]{};
        int geom_iter_pos_des{};
        int geom_iter_pos_{};
    };
}

#endif
