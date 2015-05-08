#include <sot_fcl_distance_computation/DistanceComputation.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <sot_fcl_distance_computation/conversions.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <kdl/frames_io.hpp>

using namespace distance;

DistanceComputation::DistanceComputation(
		const sensor_msgs::JointStateConstPtr& msg) {
	if (!model_.initParam("robot_description")) {
		ROS_ERROR("Failed to parse urdf file");
	}
	ROS_INFO("Successfully parsed urdf file");

	if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
		ROS_ERROR("Failed to construct kdl tree");
	} else {
		ROS_DEBUG_STREAM("Successfully created kdl tree");
		std::cerr << "number of joint: " << tree_.getNrOfJoints() << std::endl;
		std::cerr << "number of links: " << tree_.getNrOfSegments()
				<< std::endl;
	}

	robot_fk_.reset(new KDL::TreeFkSolverPos_recursive(tree_));
	base_frame_ = tree_.getRootSegment()->second.segment.getName();
	std::cerr << "root/base frame: " << base_frame_ << std::endl;

	updateJointStates(msg);
	parseCollisionObjects(model_);

	joints_kdl.resize(joint_states_.size());
    model_.getLinks(links_);
}

std::string DistanceComputation::getBaseFrame() const {
	return this->base_frame_;
}

DistanceComputation::~DistanceComputation() {
}

void DistanceComputation::printLinks() {
	ROS_INFO("Printing Link Information");

    ROS_INFO("Total Amount of Links: %d", links_.size());
    typedef std::vector<boost::shared_ptr<urdf::Link> >::iterator it_type;
    for (it_type iterator = links_.begin();
            iterator != links_.end(); iterator++) {
        ROS_INFO("Link: %s ", (*iterator)->name.c_str());
	}
}

void DistanceComputation::printJoints() {
	ROS_INFO("Printing Joint Information");
	ROS_INFO("Total Amount of Joints: %d", joint_states_.size());
	for (unsigned int var = 0; var < joint_states_.size(); ++var) {
		ROS_INFO("joint order: %s, with value: %f", joint_names_[var].c_str(),
				joint_states_[var]);
	}
}


void DistanceComputation::updateJointStates(
		const sensor_msgs::JointStateConstPtr& msg) {
    joint_states_.clear();
    for (unsigned int i = 0; i < msg->name.size(); ++i) {
        joint_states_.push_back(msg->position[i]);
        joint_names_.push_back(msg->name[i]);
    }
}

bool DistanceComputation::parseCollisionObjects(
		urdf::Model const &robot_model) {
	ROS_DEBUG_STREAM("parsing collision objects");
	ROS_DEBUG_STREAM("number of joints of tree"<<tree_.getNrOfJoints());

    typedef std::vector<boost::shared_ptr<urdf::Link> >::iterator it_type;
    for (it_type iterator = links_.begin();
            iterator != links_.end(); iterator++) {

        boost::shared_ptr<urdf::Link> link = *iterator;

		if (link->collision) {
			if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {

				boost::shared_ptr<urdf::Cylinder> collisionGeometry =
						boost::dynamic_pointer_cast<urdf::Cylinder>(
								link->collision->geometry);

				 boost::shared_ptr<fcl::Capsule>capsule = boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(collisionGeometry->radius, collisionGeometry->length));

				boost::shared_ptr<fcl::CollisionObject> collision_object(
						new fcl::CollisionObject(capsule));

				collision_objects_[link->name] = collision_object;
				capsules_[link->name] = capsule;

                // Store the transformation of the center of the CollisionShape (URDF)
				// Apply this transformation later in the update cycle to have the shape at the same origin as the link
                convert(link->collision->origin, link_T_shape[link->name]);
				ROS_INFO("adding capsule for %s",link->name.c_str() );
			}
            if (link->collision->geometry->type == urdf::Geometry::SPHERE) {

                boost::shared_ptr<urdf::Sphere> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Sphere>(
                                link->collision->geometry);

                 boost::shared_ptr<fcl::Sphere> sphere = boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(collisionGeometry->radius));

                boost::shared_ptr<fcl::CollisionObject> collision_object(
                        new fcl::CollisionObject(sphere));

                collision_objects_[link->name] = collision_object;

                // Store the transformation of the center of the CollisionShape (URDF)
                // Apply this transformation later in the update cycle to have the shape at the same origin as the link
                convert(link->collision->origin, link_T_shape[link->name]);
                ROS_INFO("adding sphere for %s",link->name.c_str() );
            }
            if (link->collision->geometry->type == urdf::Geometry::BOX) {

                boost::shared_ptr<urdf::Box> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Box>(
                                link->collision->geometry);

                 boost::shared_ptr<fcl::Box> box = boost::shared_ptr<fcl::Box>(new fcl::Box(collisionGeometry->dim.x,
                                                                                            collisionGeometry->dim.y,
                                                                                            collisionGeometry->dim.z));

                boost::shared_ptr<fcl::CollisionObject> collision_object(
                        new fcl::CollisionObject(box));

                collision_objects_[link->name] = collision_object;

                // Store the transformation of the center of the CollisionShape (URDF)
                // Apply this transformation later in the update cycle to have the shape at the same origin as the link
                convert(link->collision->origin, link_T_shape[link->name]);
                ROS_INFO("adding box for %s",link->name.c_str() );
            }
            else if(link->collision->geometry->type == urdf::Geometry::MESH){

                boost::shared_ptr< ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> (link->collision->geometry);

                shapes::Mesh *mesh = shapes::createMeshFromResource(collisionGeometry->filename);

                std::vector<fcl::Vec3f> vertices;
                std::vector<fcl::Triangle> triangles;

                for(unsigned int i=0; i < mesh->vertex_count; ++i){
                    fcl::Vec3f v(mesh->vertices[3*i]*collisionGeometry->scale.x,
                                 mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                                 mesh->vertices[3*i + 2]*collisionGeometry->scale.z);

                    vertices.push_back(v);
                }

                for(unsigned int i=0; i< mesh->triangle_count; ++i){
                    fcl::Triangle t(mesh->triangles[3*i],
                                    mesh->triangles[3*i + 1],
                                    mesh->triangles[3*i + 2]);
                    triangles.push_back(t);
                }

                // add the mesh data into the BVHModel structure
                boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS> > shape(new fcl::BVHModel<fcl::OBBRSS>);
                shape->beginModel();
                shape->addSubModel(vertices, triangles);
                shape->endModel();

                boost::shared_ptr<fcl::CollisionObject> collision_object(new fcl::CollisionObject(shape));
                collision_objects_[link->name] = collision_object;
                convert(link->collision->origin, link_T_shape[link->name]);
                ROS_INFO("adding mesh for %s",link->name.c_str() );

            }
		} else {
            ROS_WARN_STREAM("Collision not defined for link %s: "<<link->name.c_str());
		}
	}
	return true;
}

fcl::DistanceResult DistanceComputation::minimum_distance(std::string linkAname,
        std::string linkBname, fcl::Vec3f& relativeP1, fcl::Vec3f& relativeP2) {

	boost::shared_ptr<urdf::Link> linkA;
	model_.getLink(linkAname, linkA);

	boost::shared_ptr<urdf::Link> linkB;
	model_.getLink(linkBname, linkB);

//	printJoints();

    KDL::Frame bl_T_linkA, bl_T_shapeA;
    KDL::Frame bl_T_linkB, bl_T_shapeB;

	if (joint_states_.size() != tree_.getNrOfJoints()) {
		ROS_ERROR_STREAM(
				"There is a mismatch between the configured joints and the passed joints in no self collision of safety layer");
		ROS_ERROR_STREAM("number of joint_states " << joint_states_.size());
		ROS_ERROR_STREAM("number of tree joints " << tree_.getNrOfJoints());

	}

	convert(joint_states_, joints_kdl);

    if(robot_fk_->JntToCart(joints_kdl, bl_T_linkA, linkAname) < 0 ||
       robot_fk_->JntToCart(joints_kdl, bl_T_linkB, linkBname) < 0)
    {
        std::cerr << "Error calling JntToCart";
        throw new std::runtime_error("Something went wrong while computing fwd kinematics");
    }

    // Get all the Frames inside the origin of the link
    bl_T_shapeA = bl_T_linkA * link_T_shape[linkAname];
    bl_T_shapeB = bl_T_linkB * link_T_shape[linkBname];

	//Check for collision
	// t1, t2 will result in the new points computed by the forward kinematic
    fcl::Transform3f fcl_bl_T_linkA, fcl_bl_T_shapeA;
    fcl::Transform3f fcl_bl_T_linkB, fcl_bl_T_shapeB;
    fcl_bl_T_linkA = kdl2fcl(bl_T_linkA);
    fcl_bl_T_shapeA = kdl2fcl(bl_T_shapeA);
    fcl_bl_T_linkB = kdl2fcl(bl_T_linkB);
    fcl_bl_T_shapeB = kdl2fcl(bl_T_shapeB);

    fcl::CollisionObject* collObj_shapeA = collision_objects_[linkAname].get();
    fcl::CollisionObject* collObj_shapeB = collision_objects_[linkBname].get();

    collObj_shapeA->setTransform(fcl_bl_T_shapeA);
    collObj_shapeB->setTransform(fcl_bl_T_shapeB);

	fcl::DistanceRequest request;
	request.gjk_solver_type = fcl::GST_INDEP;
	request.enable_nearest_points = true;

	// result will be returned via the collision result structure
	fcl::DistanceResult result;

	// perform distance test
    fcl::distance(collObj_shapeA, collObj_shapeB, request, result);

	// p1Homo, p2Homo newly computed points by FCL
	// absolutely computed w.r.t. base-frame
    fcl::Transform3f bl_pAHomo(result.nearest_points[0]);
    fcl::Transform3f bl_pBHomo(result.nearest_points[1]);

    fcl::Transform3f shapeA_pA, shapeB_pB;

    shapeA_pA = fcl_bl_T_shapeA.inverseTimes(bl_pAHomo);
    shapeB_pB = fcl_bl_T_shapeB.inverseTimes(bl_pBHomo);

    relativeP1 = shapeA_pA.getTranslation();
    relativeP2 = shapeB_pB.getTranslation();

    std::cerr << "bl_p" << linkAname << result.nearest_points[0] << std::endl;
    std::cerr << "bl_T_shape" << linkAname << collObj_shapeA->getTranslation() << "==" << bl_T_shapeA.p << std::endl;
    std::cerr << "bl_T_" << linkAname << bl_T_linkA << std::endl;
    std::cerr << linkAname << "_pA" <<relativeP1 << std::endl;

    std::cerr << "bl_p" << linkBname << result.nearest_points[1] << std::endl;
    std::cerr << "bl_T_shape" << linkBname << collObj_shapeB->getTranslation() << "==" << bl_T_shapeB.p << std::endl;
    std::cerr << "bl_T_" << linkBname << bl_T_linkB << std::endl;
    std::cerr << linkBname << "_pB" << relativeP2 << std::endl;

    std::cerr << "RESULTING DISTANCE: " << result.min_distance << std::endl;

	return result;
}
