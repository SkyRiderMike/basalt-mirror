#include "basalt/vio_viewer/vio_viewer.h"

#include "basalt/utils/vis_utils.h"

#include <basalt/vi_estimator/vio_estimator.h>

#define UNUSED(x) (void)(x)

namespace RobotA{

constexpr int UI_WIDTH = 200;

using Button = pangolin::Var<std::function<void(void)>>;

void VisualReceiver::Run(){
  while(1){
    BlockIfPaused();
    if(IsStopped()) break;
    
    printf("visual data receiver: \n");
    basalt::VioVisualizationData::Ptr data = nullptr;
    // bool result = input_vis_queue.front_pop(data);

    // if (data.get()){
    //     host_viewer->vis_map[data->t_ns] = data;
    // } 

    // if(!result) break;
  }
}

void StatesReceiver::Run(){
  while(1){
    BlockIfPaused();
    if(IsStopped()) break;
  }
}

void VioViewer::UISetup(){
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    main_display = pangolin::CreateDisplay().SetBounds(
        0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    img_view_display = pangolin::CreateDisplay()
                                           .SetBounds(0.4, 1.0, 0.0, 0.4)
                                           .SetLayout(pangolin::LayoutEqual);

    pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
        0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100, -10.0, 10.0, 0.01f,
                                    0.01f);
    plot_display.AddDisplay(*plotter);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));


    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    // while (img_view.size() < 2){
    //   std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

    //   size_t idx = img_view.size();
    //   img_view.push_back(iv);

    //   img_view_display.AddDisplay(*iv);
    //   iv->extern_draw_function = std::bind(&VioViewer::draw_image_overlay, this,
    //                                         std::placeholders::_1, idx);
    // }

    camera = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY));

    display3D = pangolin::CreateDisplay()
            .SetAspect(-640 / 480.0)
            .SetBounds(0.4, 1.0, 0.4, 1.0)
            .SetHandler(new pangolin::Handler3D(camera));

    display3D.extern_draw_function = std::bind(&VioViewer::draw_scene, this, 
                                            std::placeholders::_1);

    main_display.AddDisplay(img_view_display);
    main_display.AddDisplay(display3D);
}

void VioViewer::draw_image_overlay(pangolin::View& v, size_t cam_id) {
    
    size_t frame_id = show_frame;
    auto it = vis_map.find(vis_timestamps[frame_id]);

    if (show_obs)
    {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 0.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (it != vis_map.end() && cam_id < it->second->projections.size())
      {
        const auto &points = it->second->projections[cam_id];

        if (points.size() > 0)
        {
          double min_id = points[0][2], max_id = points[0][2];

          for (const auto &points2 : it->second->projections)
            for (const auto &p : points2)
            {
              min_id = std::min(min_id, p[2]);
              max_id = std::max(max_id, p[2]);
            }

          for (const auto &c : points)
          {
            const float radius = 6.5;

            float r, g, b;
            getcolor(c[2] - min_id, max_id - min_id, b, g, r);
            glColor3f(r, g, b);

            pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

            if (show_ids)
              pangolin::GlFont::I().Text("%d", int(c[3])).Draw(c[0], c[1]);
          }
        }

        glColor3f(1.0, 0.0, 0.0);
        pangolin::GlFont::I()
            .Text("Tracked %d points", points.size())
            .Draw(5, 20);
      }
    }
}

void VioViewer::draw_scene(pangolin::View& view){
    view.Activate(camera);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(3);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor3ubv(cam_color);
    if (!vio_t_w_i.empty()) {
      aligned_vector<Eigen::Vector3d> sub_gt(
        vio_t_w_i.begin(), vio_t_w_i.begin() + show_frame);
      pangolin::glDrawLineStrip(sub_gt);
    }

    glColor3ubv(gt_color);

    size_t frame_id = show_frame;
    int64_t t_ns = vis_timestamps[frame_id];
    auto it = vis_map.find(t_ns);

    if (it != vis_map.end()) {
    for (size_t i = 0; i < calib.T_i_c.size(); i++)
      if (!it->second->states.empty()) {
        render_camera((it->second->states.back() * calib.T_i_c[i]).matrix(),
                      2.0f, cam_color, 0.1f);
      } else if (!it->second->frames.empty()) {
        render_camera((it->second->frames.back() * calib.T_i_c[i]).matrix(),
                      2.0f, cam_color, 0.1f);
      }

    for (const auto& p : it->second->states)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);

    for (const auto& p : it->second->frames)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, pose_color, 0.1f);

    glColor3ubv(pose_color);
    pangolin::glDrawPoints(it->second->points);
  }

}

void VioViewer::draw_plots()
{
    plotter->ClearSeries();
    plotter->ClearMarkers();

    if (show_est_pos)
    {
      plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "position x", &vio_data_log);
      plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "position y", &vio_data_log);
      plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "position z", &vio_data_log);
    }

    if (show_est_vel)
    {
      plotter->AddSeries("$0", "$1", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "velocity x", &vio_data_log);
      plotter->AddSeries("$0", "$2", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "velocity y", &vio_data_log);
      plotter->AddSeries("$0", "$3", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "velocity z", &vio_data_log);
    }

    if (show_est_bg)
    {
      plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "gyro bias x", &vio_data_log);
      plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "gyro bias y", &vio_data_log);
      plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
    }

    if (show_est_ba)
    {
      plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                       pangolin::Colour::Red(), "accel bias x", &vio_data_log);
      plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                       pangolin::Colour::Green(), "accel bias y",
                       &vio_data_log);
      plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "accel bias z", &vio_data_log);
    }
    
    double t = vis_timestamps.at(show_frame) * 1e-9;
    plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
                       pangolin::Colour::White());
}

void VioViewer::Run(){
    UISetup();

    while (!pangolin::ShouldQuit()) {
      BlockIfPaused();
      if(IsStopped()){
        break;
      }

      basalt::VioVisualizationData::Ptr data = nullptr;

      // if (input_vis_queue.size() > 0) {
      //   input_vis_queue.front_pop(data);

      //   if (data != nullptr) {
      //     vis_map[data->t_ns] = data;
      //     vis_timestamps.push_back(data->t_ns);
      //   }
      // }

      // Clear screen and activate view to render into
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (show_frame.GuiChanged()) {
        size_t frame_id = static_cast<size_t>(show_frame);
        std::cout << "frame id: " << frame_id << std::endl;
      }

      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      // Draw in 3D view
      display3D.Activate(camera);
      // Render OpenGL Cube
      pangolin::glDrawColouredCube();

      // Draw in Image View display
      img_view_display.Activate();

      if (show_frame.GuiChanged()) {
        for (size_t cam_id = 0; cam_id < calib.intrinsics.size(); cam_id++) {
          // size_t frame_id = static_cast<size_t>(show_frame);
          // int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];

          // std::vector<cv::Mat> img_vec =
          //     vio_dataset->get_image_data(timestamp);

          // pangolin::GlPixFormat fmt;
          // fmt.glformat = GL_LUMINANCE;
          // fmt.gltype = GL_UNSIGNED_SHORT;
          // fmt.scalable_internal_format = GL_LUMINANCE16;

          // if (img_vec[cam_id].img.get())
          //   img_view[cam_id]->SetImage(
          //       img_vec[cam_id].img->ptr, img_vec[cam_id].img->w,
          //       img_vec[cam_id].img->h, img_vec[cam_id].img->pitch, fmt);
        }

        draw_plots();
      }

      if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() ||
          show_est_ba.GuiChanged() || show_est_bg.GuiChanged()) {
        draw_plots();
      }

      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      // img_view_display.Activate();

      // Swap frames and Process Events
      pangolin::FinishFrame();
    }
}

} // Robot A