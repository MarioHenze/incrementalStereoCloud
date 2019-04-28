#pragma once

#include <cgv/base/node.h>
#include <libs/point_cloud/ann_tree.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/provider.h>
#include <cgv/base/register.h>
#include <cgv/media/image/image.h>
#include <libs/point_cloud/gl_point_cloud_drawable.h>
#include <libs/point_cloud/neighbor_graph.h>
#include <libs/point_cloud/normal_estimator.h>

#include "lib_begin.h"

class CGV_API point_cloud_viewer :
	public cgv::base::node,
	public cgv::gui::event_handler,
	public cgv::gui::provider,
	public cgv::base::argument_handler,
	public gl_point_cloud_drawable
{
protected:
	float target_max_extent;
	void scale_to_target_extent();

	std::string file_name;

	float relative_distance_threshold;

	std::size_t show_point_start;
	std::size_t show_point_count;
	unsigned interact_point_step;
	double interact_delay;
	cgv::gui::trigger interact_trigger;
	void interact_callback(double t, double dt);

	enum InteractionState {
		IS_INTERMEDIATE_FRAME,
		IS_WAIT_INTERACTION_TO_STOP,
		IS_DRAW_FULL_FRAME,
		IS_FULL_FRAME
	} interact_state;
	void configure_subsample_controls();

	// point cloud io
	bool open(const std::string& fn);
	void auto_set_view();

	// selection
	point_cloud& ref_point_cloud() { return pc; }
public:
	point_cloud_viewer();
	std::string get_type_name() const;
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void stream_stats(std::ostream&);
	bool init(cgv::render::context& ctx);
	void init_frame(cgv::render::context& ctx);
	void draw(cgv::render::context& ctx);
	bool handle(cgv::gui::event& e);
	void handle_args(std::vector<std::string>& args);
	void stream_help(std::ostream& os);
	void on_set(void* member_ptr);
	void create_gui();
};

#include <cgv/config/lib_end.h>