#include "point_cloud_viewer.h"
#include <algorithm>
#include <libs/point_cloud/ann_tree.h>
#include <cgv/gui/animate.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/import.h>
#include <cgv/utils/file.h>
#include <cgv/render/clipped_view.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/reflect/reflect_extern.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/utils/tokenizer.h>

#define FILE_OPEN_TITLE "Open Point Cloud"
#define FILE_OPEN_FILTER "Point Clouds (apc,bpc):*.apc;*.bpc|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct|All Files:*.*"

namespace cgv {
	namespace reflect {
reflection_traits<gl_point_cloud_drawable,RTK_SELF_REFLECT,false> get_reflection_traits(const gl_point_cloud_drawable&) 
{
	return reflection_traits<gl_point_cloud_drawable,RTK_SELF_REFLECT,false>();
}
	}
}

point_cloud_viewer::point_cloud_viewer() 
{
	target_max_extent = 1.0f;

	set_name("point_cloud_viewer");

	relative_distance_threshold = 5.0f;
	show_nmls = false;
	interact_point_step = 1;
	show_point_count = 0;
	show_point_start = 0;
	interact_delay = 0.15;

	interact_state = IS_INTERMEDIATE_FRAME;

	cgv::signal::connect(interact_trigger.shoot, this, &point_cloud_viewer::interact_callback);
	interact_trigger.schedule_recuring(interact_delay);

	use_component_transformations = false;

	surfel_style.use_group_transformation = false;

	surfel_style.point_size = 4.0f;
	surfel_style.halo_color_strength = 0.0f;
	surfel_style.percentual_halo_width = 25.0f;
	surfel_style.blend_points = true;
	surfel_style.blend_width_in_pixel = 1.0f;
	sort_points = true;
}

void point_cloud_viewer::interact_callback(double t, double dt)
{
	if (interact_state == IS_FULL_FRAME || interact_state == IS_DRAW_FULL_FRAME)
		return;

	if (interact_state == IS_INTERMEDIATE_FRAME)
		interact_state = IS_WAIT_INTERACTION_TO_STOP;
	else {
		interact_state = IS_DRAW_FULL_FRAME;
		post_redraw();
	}
}

bool point_cloud_viewer::init(cgv::render::context& ctx)
{
	if (!gl_point_cloud_drawable::init(ctx))
		return false;
	// set white background 
	get_root()->set("bg_index", 4);

	return true;
}

// more per frame initialization
void point_cloud_viewer::init_frame(cgv::render::context& ctx)
{
	gl_point_cloud_drawable::init_frame(ctx);
}



void point_cloud_viewer::draw(cgv::render::context& ctx)
{
	if (pc.get_nr_points() == 0)
		return;

	if (interact_state != IS_DRAW_FULL_FRAME)
		std::swap(show_point_step, interact_point_step);

	gl_point_cloud_drawable::draw(ctx);

	if (interact_state != IS_DRAW_FULL_FRAME) {
		std::swap(show_point_step, interact_point_step);
		interact_state = IS_INTERMEDIATE_FRAME;
	}
	else
		interact_state = IS_FULL_FRAME;
}

bool point_cloud_viewer::open(const std::string& fn)
{
	if (!read(fn)) {
		cgv::gui::message(std::string("could not read ") + fn);
		return false;
	}
	file_name = fn;
	update_member(&file_name);
	
	// inform clipped view on extent of opened point cloud
	if (!view_ptr)
		view_ptr = find_view_as_node();
	cgv::render::clipped_view* clipped_view_ptr = dynamic_cast<cgv::render::clipped_view*>(view_ptr);
	if (clipped_view_ptr) {
		box3 B = pc.box();
		clipped_view_ptr->set_scene_extent(B);
	}
	// update illumination mode
	if (!pc.has_normals() != surfel_style.illumination_mode == cgv::render::IM_OFF) {
		if (pc.has_normals())
			surfel_style.illumination_mode = cgv::render::IM_ONE_SIDED;
		else
			surfel_style.illumination_mode = cgv::render::IM_OFF;
		update_member(&surfel_style.illumination_mode);
	}
	show_point_end = pc.get_nr_points();
	show_point_begin = 0;
	update_member(&show_point_begin);
	update_member(&show_point_end);

	show_point_count = pc.get_nr_points();
	update_member(&show_point_count);

	interact_point_step = std::max((unsigned)(show_point_count / 1000000), 1u);
	nr_draw_calls = std::max((unsigned)(show_point_count / 1000000), 1u);
	update_member(&interact_point_step);
	update_member(&nr_draw_calls);

	configure_subsample_controls();

	post_redraw();
	return true;
}

void point_cloud_viewer::configure_subsample_controls()
{
	if (find_control(show_point_begin)) {
		find_control(show_point_begin)->set("max", show_point_end);
		find_control(show_point_end)->set("min", show_point_begin);
		find_control(show_point_end)->set("max", pc.get_nr_points());
		find_control(show_point_count)->set("max", pc.get_nr_points());
		find_control(show_point_start)->set("max", pc.get_nr_points() - show_point_count);
	}
}


std::string point_cloud_viewer::get_type_name() const
{
	return "point_cloud_viewer";
}

bool point_cloud_viewer::self_reflect(cgv::reflect::reflection_handler& srh)
{
	if (srh.reflect_member("file_name", file_name) &&
		srh.reflect_member("interact_point_step", interact_point_step) &&
		srh.reflect_member("surfel_style", surfel_style) &&
		srh.reflect_member("normal_style", normal_style) &&
		srh.reflect_member("box_style", box_style) &&
		srh.reflect_member("box_wire_style", box_wire_style) &&
		srh.reflect_member("show_points", show_points) &&
		srh.reflect_member("show_nmls", show_nmls) &&
		srh.reflect_member("show_boxes", show_boxes) &&
		srh.reflect_member("show_box", show_box))
		return true;
	return false;
}


void point_cloud_viewer::on_set(void* member_ptr)
{
	if (member_ptr == &file_name) {
		std::cout << "file_name set to " << file_name << std::endl;
		open(file_name);
	}
	if (member_ptr == &show_point_start) {
		show_point_begin = show_point_start;
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_begin);
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_count) {
		if (show_point_start + show_point_count > pc.get_nr_points()) {
			show_point_start = pc.get_nr_points() - show_point_count;
			show_point_begin = show_point_start;
			update_member(&show_point_begin);
			update_member(&show_point_start);
		}
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_begin) {
		show_point_start = show_point_begin;
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_start);
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_end) {
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &interact_delay) {
		interact_trigger.stop();
		interact_trigger.schedule_recuring(interact_delay);
	}
	if (member_ptr == &use_component_colors) {
		surfel_style.use_group_color = use_component_colors;
		update_member(&surfel_style.use_group_color);
		box_style.use_group_color = use_component_colors;
		update_member(&box_style.use_group_color);
		box_wire_style.use_group_color = use_component_colors;
		update_member(&box_wire_style.use_group_color);
	}
	if (member_ptr == &use_component_transformations) {
		surfel_style.use_group_transformation = use_component_transformations;
		update_member(&surfel_style.use_group_transformation);
		box_style.use_group_transformation = use_component_transformations;
		update_member(&box_style.use_group_transformation);
		box_wire_style.use_group_transformation = use_component_transformations;
		update_member(&box_wire_style.use_group_transformation);
	}
	update_member(member_ptr);
	post_redraw();
}

void point_cloud_viewer::auto_set_view()
{
	if (pc.get_nr_points() == 0 || !view_ptr)
		return;
	cgv::gui::animate_with_rotation(view_ptr->ref_view_up_dir(), dvec3(0,1,0), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_geometric_blend(view_ptr->ref_y_extent_at_focus(), 1.5*pc.box().get_extent()(1), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_linear_blend(view_ptr->ref_focus(), dvec3(pc.box().get_center()), 0.5)->set_base_ptr(this);
	post_redraw();
}

bool point_cloud_viewer::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case '=' :
			case '+' :
				if (ke.get_modifiers() == 0) {
					surfel_style.point_size += 1;
					on_set(&surfel_style.point_size);
					return true;
				}
				else
					break;
			case '-' :
				if (ke.get_modifiers() == 0) {
					if (surfel_style.point_size > 0) {
						surfel_style.point_size -= 1;
						if (surfel_style.point_size < 1)
							surfel_style.point_size = 1;
						on_set(&surfel_style.point_size);
					}
					return true;
				}
				else
					break;
			case 'P' :
				if (ke.get_modifiers() == 0) {
					show_points = !show_points;
					on_set(&show_points);
					return true;
				}
				else
					break;
			case 'C' :
				if (ke.get_modifiers() == 0) {
					if (surfel_style.map_color_to_material == cgv::render::MS_FRONT_AND_BACK)
                        surfel_style.map_color_to_material = cgv::render::CM_NONE;
					else
						++(int&)surfel_style.map_color_to_material;
					on_set(&surfel_style.map_color_to_material);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					if (surfel_style.culling_mode == cgv::render::CM_FRONTFACE)
						surfel_style.culling_mode = cgv::render::CM_OFF;
					else
						++(int&)surfel_style.culling_mode;
					on_set(&surfel_style.culling_mode);
				}
				else
					break;
				return true;
			case 'B' :
				if (ke.get_modifiers() == 0) {
					surfel_style.blend_points = !surfel_style.blend_points;
					on_set(&surfel_style.blend_points);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					show_boxes = !show_boxes;
					on_set(&show_boxes);
				}
				else
					break;
				return true;
			case 'N' :
				if (ke.get_modifiers() == 0) {
					show_nmls = !show_nmls;
					on_set(&show_nmls);
					return true;
				}
				break;
			case 'I' :
				if (ke.get_modifiers() == 0) {
					if (surfel_style.illumination_mode == cgv::render::IM_TWO_SIDED)
						surfel_style.illumination_mode = cgv::render::IM_OFF;
					else
						++(int&)surfel_style.illumination_mode;
					on_set(&surfel_style.illumination_mode);
					return true;
				}
				break;
			case 'O' :
				if (ke.get_modifiers() == 0) {
					surfel_style.orient_splats = !surfel_style.orient_splats;
					on_set(&surfel_style.orient_splats);
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_CTRL) {
					std::string fn = cgv::gui::file_open_dialog(FILE_OPEN_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						open(fn);
					return true;
				}
				break;
			case 'S' :
				if (ke.get_modifiers() == 0) {
					sort_points = !sort_points;
					on_set(&sort_points);
					return true;
				}
				break;
			case cgv::gui::KEY_Space :
				if (ke.get_modifiers() == 0) {
					if (pc.get_nr_points() > 0)
						auto_set_view();
					return true;
				}
				break;
			}
		}
	}
	else {
		if (e.get_kind() == cgv::gui::EID_MOUSE) {
			if (e.get_flags() == cgv::gui::EF_DND) {
				cgv::gui::mouse_event& me = static_cast<cgv::gui::mouse_event&>(e);
				if (me.get_action() == cgv::gui::MA_RELEASE) {
					open(me.get_dnd_text());
				}
				return true;
			}
		}
	}
	return false;
}

void point_cloud_viewer::handle_args(std::vector<std::string>& args)
{
	for (unsigned ai = 0; ai < args.size(); ++ai) {
		if (cgv::utils::file::exists(args[ai])) {
			if (open(args[ai])) {
				args.erase(args.begin() + ai);
				--ai;
				break;
			}
		}
	}
}


void point_cloud_viewer::stream_help(std::ostream& os)
{
	os << "PCV: open (Ctrl-O), toggle <p>oints, <n>ormals, <b>ox, <g>graph, <i>llum, <s>ort; <Space> ... view all" << std::endl;
}

void point_cloud_viewer::stream_stats(std::ostream& os)
{
	os << "PCV: #P=" << pc.get_nr_points() 
		<< ", #N=" << (pc.has_normals()?pc.get_nr_points():0) 
		<< ", #C=" << (pc.has_colors() ? pc.get_nr_points() : 0)
		<< ", B=" << pc.box().get_center() << "<" << pc.box().get_extent() << ">" << std::endl;
}

void point_cloud_viewer::scale_to_target_extent()
{
	float max_extent = pc.box().get_extent()(pc.box().get_max_extent_coord_index());
	float scale = target_max_extent / max_extent;
	point_cloud::AMat M;
	M.zeros();
	M(0, 0) = M(1, 1) = M(2, 2) = scale;
	pc.transform(M);
	if (pc.has_component_transformations()) {
		for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci)
			pc.component_translation(ci) *= scale;
	}
	post_redraw();
}

void point_cloud_viewer::create_gui()
{
	add_decorator("Point Cloud Viewer", "heading", "level=2");
	add_member_control(this, "target_max_extent", target_max_extent, "value_slider", "min=0.1;max=1000;log=true;ticks=true'");
	connect_copy(add_button("scale to target extent")->click, cgv::signal::rebind(this, &point_cloud_viewer::scale_to_target_extent));
	add_gui("file_name", file_name, "file_name", "title='" FILE_OPEN_TITLE "';filter='" FILE_OPEN_FILTER "'");
	bool show = begin_tree_node("points", show_points, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_points, "toggle", "w=50");
	if (show) {
		align("\a");
			add_member_control(this, "nr_draw_calls", nr_draw_calls, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact step", interact_point_step, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact delay", interact_delay, "value_slider", "min=0.01;max=1;log=true;ticks=true");
			add_member_control(this, "show step", show_point_step, "value_slider", "min=1;max=20;log=true;ticks=true");
			add_decorator("range control", "heading", "level=3");
			add_member_control(this, "begin", show_point_begin, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "end", show_point_end, "value_slider", "min=0;max=10;ticks=true");
			add_decorator("window control", "heading", "level=3");
			add_member_control(this, "start", show_point_start, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "width", show_point_count, "value_slider", "min=0;max=10;ticks=true");
			configure_subsample_controls();
			add_member_control(this, "sort_points", sort_points, "check");
			add_gui("surfel_style", surfel_style);
		align("\b");
		end_tree_node(show_points);
	}
	show = begin_tree_node("components", pc.components, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", surfel_style.use_group_color, "toggle", "w=50");
	if (show) {
		align("\a");
		if (begin_tree_node("component colors", pc.component_colors, false)) {
			align("\a");
			for (unsigned i = 0; i < pc.component_colors.size(); ++i) {
				add_member_control(this,
					pc.component_name(i).empty() ? std::string("C") + cgv::utils::to_string(i) : pc.component_name(i),
					pc.component_colors[i]);
			}
			align("\b");
			end_tree_node(pc.component_colors);
		}
		if (begin_tree_node("group transformations", pc.component_translations, false)) {
			align("\a");
			for (unsigned i = 0; i < pc.component_translations.size(); ++i) {
				add_decorator(pc.component_name(i).empty() ? std::string("C") + cgv::utils::to_string(i) : pc.component_name(i), "heading", "level=3");
				add_decorator("translation", "heading", "level=4");
				add_gui(std::string("T") + cgv::utils::to_string(i), pc.component_translations[i]);
				add_decorator("rotation", "heading", "level=4");
				add_gui(std::string("Q") + cgv::utils::to_string(i), (HVec&)pc.component_rotations[i], "direction");
			}
			align("\b");
			end_tree_node(pc.component_translations);
		}
		align("\b");
		end_tree_node(pc.components);
	}
	show = begin_tree_node("normals", show_nmls, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_nmls, "toggle", "w=50");
	if (show) {
		add_gui("normal_style", normal_style);
		end_tree_node(show_nmls);
	}
	show = begin_tree_node("box", show_box, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_box, "toggle", "w=50");
	if (show) {
		add_member_control(this, "show", show_boxes, "toggle", "w=50");
		add_gui("color", box_color);
		add_gui("box_style", box_style);
		add_gui("box_wire_style", box_wire_style);
		end_tree_node(show_box);
	}
}

#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
//extern cgv::base::object_registration<point_cloud_viewer> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
//extern cgv::base::registration_order_definition dro("stereo_view_interactor;point_cloud_viewer");
#endif


