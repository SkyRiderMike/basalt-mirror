#pragma once

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <basalt/utils/vis_utils.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/vi_estimator/vio_estimator.h>

#include <toolbox/utils/threading.h>
#include <toolbox/utils/thread_safe_queue.h>
#include <toolbox/utils/alignment.h>


namespace RobotA{

class VioViewer;

class VisualReceiver : public utils::Thread{
public:
    VisualReceiver(VioViewer* host_ptr) : host_viewer(host_ptr) {
        CHECK(host_viewer) << "can't give a nullptr to the host";
    }

    virtual ~VisualReceiver() = default;
    
    virtual void Run() override;
    virtual void Stop() override {input_vis_queue.shut_down(); Thread::Stop();}

    utils::ThreadSafeQueue<basalt::VioVisualizationData::Ptr> input_vis_queue;

private:
    VioViewer* host_viewer = nullptr;
};

class StatesReceiver : public utils::Thread{
public:
    StatesReceiver(VioViewer* host_ptr) : host_viewer(host_ptr) {
        CHECK(host_viewer) << "can't give a nullptr to the host";
    }

    virtual ~StatesReceiver() = default;
    
    virtual void Run() override;
    virtual void Stop() override {input_state_queue.shut_down(); Thread::Stop();}

    utils::ThreadSafeQueue<basalt::PoseVelBiasState::Ptr> input_state_queue;

private:
    VioViewer* host_viewer = nullptr;
};

class VioViewer : public utils::Thread {
public:
    VioViewer(): 
        // ui elements ...
        // show_frame("ui.show_frame", 0, 0, 1500),
        // show_flow("ui.show_flow", true, false, true),
        // show_obs("ui.show_obs", true, false, true),
        // show_ids("ui.show_ids", false, false, true),
        show_est_pos("ui.show_est_pos", true, false, true),
        show_est_vel("ui.show_est_vel", false, false, true),
        show_est_bg("ui.show_est_bg", false, false, true),
        show_est_ba("ui.show_est_ba", false, false, true),
        show_gt("ui.show_gt", true, false, true),
        continue_btn("ui.continue", false, false, true),
        continue_fast("ui.continue_fast", true, false, true),
        follow("ui.follow", true, false, true),
        visual_receiver_(new VisualReceiver(this)),
        states_receiver_(new StatesReceiver(this))
    {}
    
    virtual ~VioViewer() = default;

    void UISetup();

    virtual void Run() override;
    
    virtual void Start() override {
        visual_receiver_->Start();
        states_receiver_->Start();
        Thread::Start();
    }

    
    void draw_image_overlay(pangolin::View& v, size_t cam_id);

    void draw_scene(pangolin::View& view);

    void draw_plots();

    // VIO variables
    basalt::Calibration<double> calib;

    friend class VisualReceiver;
    friend class StatesReceiver;

private:
    pangolin::OpenGlRenderState camera;

    std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;
    std::vector<int64_t> vis_timestamps;

    // container for history data.
    std::vector<int64_t> vio_t_ns;
    aligned_vector<Eigen::Vector3d> vio_t_w_i;
    aligned_vector<Sophus::SE3d> vio_T_w_i;

    std::vector<int64_t> gt_t_ns;
    aligned_vector<Eigen::Vector3d> gt_t_w_i;

    // displays and UI.
    pangolin::View img_view_display;
    pangolin::View main_display;
    pangolin::View display3D;

    // curve plotter.
    pangolin::Plotter* plotter = nullptr;

    pangolin::DataLog imu_data_log;
    pangolin::DataLog vio_data_log;
    
    // ui elements....
    pangolin::Var<int>  show_frame = pangolin::Var<int>("ui.show_frame", 0, 0, 1500);

    pangolin::Var<bool> show_flow = pangolin::Var<bool>("ui.show_flow", true, false, true);
    pangolin::Var<bool> show_obs = pangolin::Var<bool>("ui.show_obs", true, false, true);
    pangolin::Var<bool> show_ids = pangolin::Var<bool>("ui.show_ids", false, false, true);

    pangolin::Var<bool> show_est_pos;
    pangolin::Var<bool> show_est_vel;
    pangolin::Var<bool> show_est_bg;
    pangolin::Var<bool> show_est_ba;
    pangolin::Var<bool> show_gt;

    pangolin::Var<bool> continue_btn;
    pangolin::Var<bool> continue_fast;
    pangolin::Var<bool> follow;

    // associate threads
    std::unique_ptr<Thread> visual_receiver_;
    std::unique_ptr<Thread> states_receiver_;
};
}