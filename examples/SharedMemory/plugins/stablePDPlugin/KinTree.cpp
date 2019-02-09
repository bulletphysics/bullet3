#include "KinTree.h"

#include <iostream>

#include "RBDUtil.h"

#ifdef USE_JSON
#include "FileUtil.h"
#endif


const int cKinTree::gPosDim = 3;
const int cKinTree::gRotDim = 4;
const int cKinTree::gRootDim = gPosDim + gRotDim;
const int cKinTree::gInvalidJointID = -1;

// Json keys
const std::string gJointTypeNames[cKinTree::eJointTypeMax] =
{
	"revolute",
	"planar",
	"prismatic",
	"fixed",
	"spherical",
	"none"
};

const std::string gJointsKey = "Joints";
const std::string gJointDescKeys[cKinTree::eJointDescMax] = 
{
	"Type",
	"Parent",
	"AttachX",
	"AttachY",
	"AttachZ",
	"AttachThetaX",
	"AttachThetaY",
	"AttachThetaZ",
	"LimLow0",
	"LimLow1",
	"LimLow2",
	"LimHigh0",
	"LimHigh1",
	"LimHigh2",
	"TorqueLim",
	"ForceLim",
	"IsEndEffector",
	"DiffWeight",
	"Offset"
};

const std::string gBodyDefsKey = "BodyDefs";
const std::string gBodyDescKeys[cKinTree::eBodyParamMax] =
{
	"Shape",
	"Mass",
	"ColGroup",
	"EnableFallContact",
	"AttachX",
	"AttachY",
	"AttachZ",
	"AttachThetaX",
	"AttachThetaY",
	"AttachThetaZ",
	"Param0",
	"Param1",
	"Param2",
	"ColorR",
	"ColorG",
	"ColorB",
	"ColorA"
};

const std::string gDrawShapeDefsKey = "DrawShapeDefs";
const std::string gDrawShapeDescKeys[cKinTree::eDrawShapeParamMax] =
{
	"Shape",
	"ParentJoint",
	"AttachX",
	"AttachY",
	"AttachZ",
	"AttachThetaX",
	"AttachThetaY",
	"AttachThetaZ",
	"Param0",
	"Param1",
	"Param2",
	"ColorR",
	"ColorG",
	"ColorB",
	"ColorA",
	"MeshID"
};

int cKinTree::GetRoot(const Eigen::MatrixXd& joint_desc)
{
	// this should always be true right?
	return 0;
}

void cKinTree::FindChildren(const Eigen::MatrixXd& joint_desc, int joint_id, Eigen::VectorXi& out_children)
{
	const int max_size = 128;
	int children_buffer[max_size];
	int num_children = 0;
	int num_joints = GetNumJoints(joint_desc);

	for (int i = 0; i < num_joints; ++i)
	{
		int parent = GetParent(joint_desc, i);
		if (parent == joint_id)
		{
			children_buffer[num_children] = i;
			++num_children;

			if (num_children >= max_size)
			{
				printf("Too many children, max = %i", max_size);
				assert(false);
				return;
			}
		}
	}

	out_children.resize(num_children);
	for (int i = 0; i < num_children; ++i)
	{
		out_children[i] = children_buffer[i];
	}
}

#ifdef USE_JSON
bool cKinTree::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs)
{
	bool succ = true;
	std::string str;
	
	std::ifstream f_stream(char_file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ = false;
		if (!root[gBodyDefsKey].isNull())
		{
			Json::Value body_defs = root.get(gBodyDefsKey, 0);
			int num_bodies = body_defs.size();

			succ = true;
			out_body_defs.resize(num_bodies, eBodyParamMax);
			for (int b = 0; b < num_bodies; ++b)
			{
				tBodyDef curr_def = BuildBodyDef();
				Json::Value body_json = body_defs.get(b, 0);
				bool succ_def = ParseBodyDef(body_json, curr_def);

				if (succ)
				{
					out_body_defs.row(b) = curr_def;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
	}
	
	if (!succ)
	{
		printf("Failed to load body definition from %s\n", char_file.c_str());
		succ = false;
	}

	return succ;
}

bool cKinTree::ParseBodyDef(const Json::Value& root, cKinTree::tBodyDef& out_def)
{
	std::string shape_str = root.get(gBodyDescKeys[eBodyParamShape], "").asString();
	cShape::eShape shape;
	bool succ = cShape::ParseShape(shape_str, shape);
	if (succ)
	{
		out_def(eBodyParamShape) = static_cast<double>(static_cast<int>(shape));
	}

	for (int i = 0; i < eBodyParamMax; ++i)
	{
		const std::string& curr_key = gBodyDescKeys[i];
		if (!root[curr_key].isNull()
			&& root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_def(i) = val;
		}
	}

	return succ;
}

bool cKinTree::LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs)
{
	bool succ = true;
	std::string str;

	std::ifstream f_stream(char_file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gDrawShapeDefsKey].isNull())
		{
			Json::Value shape_defs = root.get(gDrawShapeDefsKey, 0);
			int num_shapes = shape_defs.size();

			succ = true;
			out_draw_defs.resize(num_shapes, eDrawShapeParamMax);
			for (int b = 0; b < num_shapes; ++b)
			{
				tDrawShapeDef curr_def = BuildDrawShapeDef();
				Json::Value shape_json = shape_defs.get(b, 0);
				bool succ_def = ParseDrawShapeDef(shape_json, curr_def);

				if (succ)
				{
					out_draw_defs.row(b) = curr_def;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
	}

	if (!succ)
	{
		printf("Failed to load draw shape definition from %s\n", char_file.c_str());
		assert(false);
	}

	return succ;
}

bool cKinTree::ParseDrawShapeDef(const Json::Value& root, tDrawShapeDef& out_def)
{
	std::string shape_str = root.get(gDrawShapeDescKeys[eDrawShapeShape], "").asString();
	cShape::eShape shape;
	bool succ = cShape::ParseShape(shape_str, shape);
	if (succ)
	{
		out_def(eDrawShapeShape) = static_cast<double>(static_cast<int>(shape));
	}

	for (int i = 0; i < eDrawShapeParamMax; ++i)
	{
		const std::string& curr_key = gDrawShapeDescKeys[i];
		if (!root[curr_key].isNull()
			&& root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_def(i) = val;
		}
	}

	return succ;
}



bool cKinTree::Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat)
{
	bool succ = false;

	if (!root[gJointsKey].isNull())
	{
		Json::Value joints = root[gJointsKey];
		int num_joints = joints.size();

		out_joint_mat.resize(num_joints, eJointDescMax);

		for (int j = 0; j < num_joints; ++j)
		{
			tJointDesc curr_joint_desc = tJointDesc::Zero();

			Json::Value joint_json = joints.get(j, 0);
			succ = ParseJoint(joint_json, curr_joint_desc);
			if (succ)
			{
				out_joint_mat.row(j) = curr_joint_desc;
			}
			else
			{
				printf("Failed to parse joint %i\n", j);
				return false;
			}
		}

		for (int j = 0; j < num_joints; ++j)
		{
			const auto& curr_desc = out_joint_mat.row(j);
			int parent_id = static_cast<int>(curr_desc(eJointDescParent));
			if (parent_id >= j)
			{
				printf("Parent id must be < child id, parent id: %i, child id: %i\n", parent_id, j);
				out_joint_mat.resize(0, 0);
				assert(false);

				return false;
			}

			out_joint_mat.row(j) = curr_desc;
		}

		PostProcessJointMat(out_joint_mat);
	}

	return succ;
}
bool cKinTree::ParseJoint(const Json::Value& root, tJointDesc& out_joint_desc)
{
	out_joint_desc = BuildJointDesc();
	eJointType joint_type = eJointTypeNone;
	const Json::Value& type_json = root[gJointDescKeys[eJointDescType]];
	if (type_json.isNull())
	{
		printf("No joint type specified\n");
	}
	else
	{
		std::string type_str = type_json.asString();
		ParseJointType(type_str, joint_type);
		out_joint_desc[eJointDescType] = static_cast<double>(static_cast<int>(joint_type));
	}

	for (int i = 0; i < eJointDescMax; ++i)
	{
		if (i != eJointDescType)
		{
			const std::string& key = gJointDescKeys[i];
			if (!root[key].isNull())
			{
				out_joint_desc[i] = root[key].asDouble();
			}
		}
	}
	return true;
}


std::string cKinTree::BuildJointMatJson(const Eigen::MatrixXd& joint_mat)
{
	std::string json = "";
	json += "{\n\"" + gJointsKey + "\":\n[\n";

	int num_joints = GetNumJoints(joint_mat);
	for (int j = 0; j < num_joints; ++j)
	{
		tJointDesc curr_desc = joint_mat.row(j);
		std::string joint_json = BuildJointJson(j, curr_desc);

		if (j != 0)
		{
			json += ",\n";
		}

		json += joint_json;
	}

	json += "\n]\n}";
	return json;
}

std::string cKinTree::BuildJointJson(int id, const tJointDesc& joint_desc)
{
	std::string json = "";
	json += "{\n";
	json += "\"ID\": " + std::to_string(id);
	for (int i = 0; i < eJointDescMax; ++i)
	{
		std::string param_name = gJointDescKeys[i];
		if (i == eJointDescType)
		{
			json += ",\n";
			int type_id = static_cast<int>(joint_desc[i]);
			std::string type_str = gJointTypeNames[type_id];
			json += "\"" + param_name + "\": \"" + type_str + "\"";
		}
		else if (i != eJointDescParamOffset)
		{
			double val = joint_desc[i];
			if (!std::isfinite(val) && (i == eJointDescTorqueLim || i == eJointDescForceLim))
			{
				continue;
			}

			if (i != 0)
			{
				json += ",\n";
			}

			json += "\"" + param_name + "\": " + std::to_string(val);
		}
	}

	json += "\n}";
	return json;
}
#endif //USE_JSON

tVector cKinTree::CalcBodyPartPos(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id)
{
	assert(IsValidBody(body_defs, part_id));
	tMatrix body_joint_trans = BodyJointTrans(body_defs, part_id);
	tMatrix joint_to_world_trans = JointWorldTrans(joint_mat, state, part_id);
	
	tVector attach_pt = joint_to_world_trans * body_joint_trans.col(3);
	attach_pt[3] = 0;
	return attach_pt;
}

tVector cKinTree::CalcBodyPartVel(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int part_id)
{
	tVector attach_pt = cKinTree::GetBodyAttachPt(body_defs, part_id);
	return CalcWorldVel(joint_mat, state, vel, part_id, attach_pt);
}

cShape::eShape cKinTree::GetBodyShape(const Eigen::MatrixXd& body_defs, int part_id)
{
	cShape::eShape shape = static_cast<cShape::eShape>(static_cast<int>(body_defs(part_id, cKinTree::eBodyParamShape)));
	return shape;
}

tVector cKinTree::GetBodyAttachPt(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	tVector attach_pt = tVector(def(eBodyParamAttachX), def(eBodyParamAttachY), def(eBodyParamAttachZ), 0);
	return attach_pt;
}

tVector cKinTree::GetBodyAttachTheta(const Eigen::MatrixXd& body_defs, int part_id)
{
	tVector attach_theta = tVector(body_defs(part_id, eBodyParamAttachThetaX),
									body_defs(part_id, eBodyParamAttachThetaY),
									body_defs(part_id, eBodyParamAttachThetaZ), 0);
	return attach_theta;
}

void cKinTree::GetBodyRotation(const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta)
{
	tVector theta = GetBodyAttachTheta(body_defs, part_id);
	cMathUtil::EulerToAxisAngle(theta, out_axis, out_theta);
}

double cKinTree::GetBodyMass(const Eigen::MatrixXd& body_defs, int part_id)
{
	double mass = body_defs(part_id, eBodyParamMass);
	return mass;
}

int cKinTree::GetBodyColGroup(const Eigen::MatrixXd& body_defs, int part_id)
{
	int col_group = static_cast<int>(body_defs(part_id, eBodyParamColGroup));
	return col_group;
}

bool cKinTree::GetBodyEnableFallContact(const Eigen::MatrixXd& body_defs, int part_id)
{
	double fall_val = body_defs(part_id, eBodyParamEnableFallContact);
	return fall_val != 0;
}

void cKinTree::SetBodyEnableFallContact(int part_id, bool enable, Eigen::MatrixXd& out_body_defs)
{
	out_body_defs(part_id, eBodyParamEnableFallContact) = (enable) ? 1 : 0;
}

tVector cKinTree::GetBodySize(const Eigen::MatrixXd& body_defs, int part_id)
{
	const tBodyDef& def = body_defs.row(part_id);
	tVector size = tVector(def(eBodyParam0), def(eBodyParam1), def(eBodyParam2), 0);
	return size;
}

tVector cKinTree::GetBodyColor(const Eigen::MatrixXd& body_defs, int part_id)
{
	const tBodyDef& def = body_defs.row(part_id);
	tVector col = tVector(def(eBodyColorR), def(eBodyColorG), def(eBodyColorB), def(eBodyColorA));
	return col;
}

double cKinTree::CalcTotalMass(const Eigen::MatrixXd& body_defs)
{
	double total_mass = 0;
	for (int i = 0; i < body_defs.rows(); ++i)
	{
		if (IsValidBody(body_defs, i))
		{
			double mass = cKinTree::GetBodyMass(body_defs, i);
			total_mass += mass;
		}
	}
	return total_mass;
}

bool cKinTree::IsValidBody(const Eigen::MatrixXd& body_defs, int part_id)
{
	cShape::eShape shape = GetBodyShape(body_defs, part_id);
	if (shape != cShape::eShapeNull)
	{
		return true;
	}
	return false;
}

tVector cKinTree::GetBodyLocalCoM(const Eigen::MatrixXd& body_defs, int part_id)
{
	cShape::eShape shape = GetBodyShape(body_defs, part_id);
	tVector com = tVector::Zero();
	switch (shape)
	{
	case cShape::eShapeBox:
	case cShape::eShapeCapsule:
	case cShape::eShapeSphere:
	case cShape::eShapeCylinder:
		break;
	default:
		assert(false); // unsupported
		break;
	}

	return com;
}

int cKinTree::GetDrawShapeParentJoint(const tDrawShapeDef& shape)
{
	return static_cast<int>(shape[eDrawShapeParentJoint]);
}

tVector cKinTree::GetDrawShapeAttachPt(const tDrawShapeDef& shape)
{
	return tVector(shape[eDrawShapeAttachX], shape[eDrawShapeAttachY], shape[cKinTree::eDrawShapeAttachZ], 0);
}

tVector cKinTree::GetDrawShapeAttachTheta(const tDrawShapeDef& shape)
{
	return tVector(shape[eDrawShapeAttachThetaX], shape[eDrawShapeAttachThetaY], shape[eDrawShapeAttachThetaZ], 0);
}

void cKinTree::GetDrawShapeRotation(const tDrawShapeDef& shape, tVector& out_axis, double& out_theta)
{
	tVector theta = GetDrawShapeAttachTheta(shape);
	cMathUtil::EulerToAxisAngle(theta, out_axis, out_theta);
}

tVector cKinTree::GetDrawShapeColor(const tDrawShapeDef& shape)
{
	return tVector(shape[eDrawShapeColorR], shape[eDrawShapeColorG], shape[eDrawShapeColorB], shape[cKinTree::eDrawShapeColorA]);
}

int cKinTree::GetDrawShapeMeshID(const tDrawShapeDef& shape)
{
	return static_cast<int>(shape[eDrawShapeMeshID]);
}

void cKinTree::CalcBodyPartRotation(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id, tVector& out_axis, double& out_theta)
{
	tMatrix mat = BodyWorldTrans(joint_mat, body_defs, state, part_id);
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}



bool cKinTree::HasValidRoot(const Eigen::MatrixXd& joint_desc)
{
	int root = GetRoot(joint_desc);
	return root != gInvalidJointID;
}

tVector cKinTree::GetRootPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state)
{
	int root_id = GetRoot(joint_mat);
	tVector pos = tVector::Zero();
	int param_offset = GetParamOffset(joint_mat, root_id);
	pos.segment(0, gPosDim) = state.segment(param_offset, gPosDim);
	return pos;
}

void cKinTree::SetRootPos(const Eigen::MatrixXd& joint_mat, const tVector& pos, Eigen::VectorXd& out_state)
{
	int root_id = GetRoot(joint_mat);
	int param_offset = GetParamOffset(joint_mat, root_id);
	out_state.segment(param_offset, gPosDim) = pos.segment(0, gPosDim);
}

tQuaternion cKinTree::GetRootRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state)
{
	int root_id = GetRoot(joint_mat);
	int param_offset = GetParamOffset(joint_mat, root_id);
	tQuaternion rot = cMathUtil::VecToQuat(state.segment(param_offset + gPosDim, gRotDim));
	return rot;
}

void cKinTree::SetRootRot(const Eigen::MatrixXd& joint_mat, const tQuaternion& rot, Eigen::VectorXd& out_state)
{
	int root_id = GetRoot(joint_mat);
	int param_offset = GetParamOffset(joint_mat, root_id);
	out_state.segment(param_offset + gPosDim, gRotDim) = cMathUtil::QuatToVec(rot).segment(0, gRotDim);
}

tVector cKinTree::GetRootVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel)
{
	int root_id = GetRoot(joint_mat);
	tVector pos = tVector::Zero();
	int param_offset = GetParamOffset(joint_mat, root_id);
	pos.segment(0, gPosDim) = vel.segment(param_offset, gPosDim);
	return pos;
}

void cKinTree::SetRootVel(const Eigen::MatrixXd& joint_mat, const tVector& vel, Eigen::VectorXd& out_vel)
{
	int root_id = GetRoot(joint_mat);
	int param_offset = GetParamOffset(joint_mat, root_id);
	out_vel.segment(param_offset, gPosDim) = vel.segment(0, gPosDim);
}

tVector cKinTree::GetRootAngVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel)
{
	int root_id = GetRoot(joint_mat);
	tVector ang_vel = tVector::Zero();
	int param_offset = GetParamOffset(joint_mat, root_id);
	ang_vel.segment(0, gRotDim) = vel.segment(param_offset + gPosDim, gRotDim);
	return ang_vel;
}

void cKinTree::SetRootAngVel(const Eigen::MatrixXd& joint_mat, const tVector& ang_vel, Eigen::VectorXd& out_vel)
{
	int root_id = GetRoot(joint_mat);
	int param_offset = GetParamOffset(joint_mat, root_id);
	out_vel.segment(param_offset + gPosDim, gRotDim) = ang_vel.segment(0, gRotDim);
}

tVector cKinTree::CalcJointWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector pos = LocalToWorldPos(joint_mat, state, joint_id, tVector::Zero());
	return pos;
}

tVector cKinTree::LocalToWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, const tVector& attach_pt)
{
	tMatrix local_to_world_trans = JointWorldTrans(joint_mat, state, parent_id);
	tVector pos = attach_pt;
	pos[3] = 1;
	pos = local_to_world_trans * pos;
	pos[3] = 0;

	return pos;
}

tQuaternion cKinTree::CalcJointWorldRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix mat = JointWorldTrans(joint_mat, state, joint_id);
	return cMathUtil::RotMatToQuaternion(mat);
}

void cKinTree::CalcJointWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id,
									tVector& out_axis, double& out_theta)
{
	tMatrix mat = JointWorldTrans(joint_mat, state, joint_id);
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

tVector cKinTree::CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id)
{
	return CalcWorldVel(joint_mat, state, vel, joint_id, tVector::Zero());
}

tVector cKinTree::CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt)
{
	cSpAlg::tSpVec sv = cRBDUtil::CalcWorldVel(joint_mat, state, vel, parent_id);
	tVector pos = cKinTree::LocalToWorldPos(joint_mat, state, parent_id, attach_pt);
	cSpAlg::tSpTrans world_to_pt = cSpAlg::BuildTrans(pos);
	sv = cSpAlg::ApplyTransM(world_to_pt, sv);
	return cSpAlg::GetV(sv);
}

tVector cKinTree::CalcJointWorldAngularVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id)
{
	return CalcWorldAngularVel(joint_mat, state, vel, joint_id, tVector::Zero());
}

tVector cKinTree::CalcWorldAngularVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt)
{
	cSpAlg::tSpVec sv = cRBDUtil::CalcWorldVel(joint_mat, state, vel, parent_id);
	tVector pos = cKinTree::LocalToWorldPos(joint_mat, state, parent_id, attach_pt);
	cSpAlg::tSpTrans world_to_pt = cSpAlg::BuildTrans(pos);
	sv = cSpAlg::ApplyTransM(world_to_pt, sv);
	return cSpAlg::GetOmega(sv);
}

int cKinTree::GetNumJoints(const Eigen::MatrixXd& joint_mat)
{
	return static_cast<int>(joint_mat.rows());
}

int cKinTree::GetNumDof(const Eigen::MatrixXd& joint_mat)
{
	int num_joints = GetNumJoints(joint_mat);
	int num_dof = cKinTree::GetParamOffset(joint_mat, num_joints - 1) + cKinTree::GetParamSize(joint_mat, num_joints - 1);
	return num_dof;
}

void cKinTree::ApplyStep(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& step, Eigen::VectorXd& out_pose)
{
	int root_id = GetRoot(joint_mat);
	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_pose += step;
}


Eigen::VectorXi cKinTree::FindJointChain(const Eigen::MatrixXd& joint_mat, int joint_beg, int joint_end)
{
	Eigen::VectorXi chain;

	const int max_length = 128;
	int chain_buffer[max_length];
	int buffer_idx = 0;

	if (joint_beg == joint_end)
	{
		Eigen::VectorXi chain(1);
		chain[0] = joint_beg;
	}

	int common_ancestor = gInvalidJointID;
	int curr_id = joint_beg;
	int end_len = 0;
	while (curr_id != gInvalidJointID)
	{
		chain_buffer[buffer_idx] = curr_id;
		++buffer_idx;

		if (buffer_idx >= max_length)
		{
			printf("Exceeded maximum chain length %i\n", max_length);
			assert(false);
			return chain;
		}

		bool is_ancestor = IsAncestor(joint_mat, joint_end, curr_id, end_len);
		if (is_ancestor)
		{
			common_ancestor = curr_id;
			break;
		}
		else
		{
			curr_id = GetParent(joint_mat, curr_id);
		}
	}

	bool found = common_ancestor != gInvalidJointID;
	// tree should always connected?
	assert(found);

	if (found)
	{
		chain.resize(buffer_idx + end_len);
		for (int i = 0; i < buffer_idx; ++i)
		{
			chain[i] = chain_buffer[i];
		}

		int idx = buffer_idx;
		curr_id = joint_end;
		while (curr_id != common_ancestor)
		{
			chain[idx] = curr_id;
			curr_id = GetParent(joint_mat, curr_id);
			++idx;
		}

		int num_flips = static_cast<int>(chain.size()) - buffer_idx;
		chain.block(buffer_idx, 0, num_flips, 1).reverseInPlace();
	}

	return chain;
}

bool cKinTree::IsAncestor(const Eigen::MatrixXd& joint_mat, int child_joint, int ancestor_joint, int& out_len)
{
	int curr_id = child_joint;
	out_len = 0;
	while (curr_id != gInvalidJointID)
	{
		if (curr_id == ancestor_joint)
		{
			return true;
		}
		else
		{
			curr_id = GetParent(joint_mat, curr_id);
			++out_len;
		}
	}
	return false;
}

double cKinTree::CalcChainLength(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXi& chain)
{
	double len = 0;
	int num_joints = static_cast<int>(chain.size());
	for (int i = 1; i < num_joints; ++i)
	{
		int curr_id = chain(i);
		int prev_id = chain(i - 1);

		if (prev_id != gInvalidJointID)
		{
			int prev_parent = GetParent(joint_mat, prev_id);
			bool is_parent = (prev_parent == curr_id);
			if (is_parent)
			{
				double curr_len = CalcLinkLength(joint_mat, prev_id);
				len += curr_len;
			}
		}

		if (curr_id != gInvalidJointID)
		{
			int curr_parent = GetParent(joint_mat, curr_id);
			bool is_child = (curr_parent == prev_id);
			if (is_child)
			{
				double curr_len = CalcLinkLength(joint_mat, curr_id);
				len += curr_len;
			}
		}
	}

	return len;
}

void cKinTree::CalcAABB(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& state, tVector& out_min, tVector& out_max)
{
	out_min[0] = std::numeric_limits<double>::infinity();
	out_min[1] = std::numeric_limits<double>::infinity();
	out_min[2] = std::numeric_limits<double>::infinity();

	out_max[0] = -std::numeric_limits<double>::infinity();
	out_max[1] = -std::numeric_limits<double>::infinity();
	out_max[2] = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumJoints(joint_desc); ++i)
	{
		tVector pos = CalcJointWorldPos(joint_desc, state, i);
		out_min = out_min.cwiseMin(pos);
		out_max = out_max.cwiseMax(pos);
	}
}

int cKinTree::GetParamOffset(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int offset = static_cast<int>(joint_mat(joint_id, eJointDescParamOffset));
	return offset;
}

int cKinTree::GetParamSize(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);
	int size = (is_root) ? gRootDim : GetJointParamSize(joint_type);
	return size;
}

int cKinTree::GetJointParamSize(eJointType joint_type)
{
	int size = 0;
	switch (joint_type)
	{
	case eJointTypeRevolute:
		size = 1;
		break;
	case eJointTypePrismatic:
		size = 1;
		break;
	case eJointTypePlanar:
		size = 3;
		break;
	case eJointTypeFixed:
		size = 0;
		break;
	case eJointTypeSpherical:
		size = 4;
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}

	return size;
}

void cKinTree::GetJointParams(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int j, Eigen::VectorXd& out_params)
{
	int offset = GetParamOffset(joint_mat, j);
	int dim = GetParamSize(joint_mat, j);
	if (dim > 0)
	{
		out_params = state.block(offset, 0, dim, 1);
	}
	else
	{
		out_params = Eigen::VectorXd::Zero(1);
	}
}

void cKinTree::SetJointParams(const Eigen::MatrixXd& joint_mat, int j, const Eigen::VectorXd& params, Eigen::VectorXd& out_state)
{
	int offset = GetParamOffset(joint_mat, j);
	int dim = GetParamSize(joint_mat, j);
	assert(dim == params.size());
	out_state.block(offset, 0, dim, 1) = params;
}

cKinTree::eJointType cKinTree::GetJointType(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	eJointType type = static_cast<eJointType>(static_cast<int>(joint_mat(joint_id, cKinTree::eJointDescType)));
	return type;
}

int cKinTree::GetParent(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int parent = static_cast<int>(joint_mat(joint_id, cKinTree::eJointDescParent));
	assert(parent < joint_id); // joints should always be ordered as such
								// since some algorithms will assume this ordering
	return parent;
}

bool cKinTree::HasParent(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int parent = GetParent(joint_mat, joint_id);
	return parent != gInvalidJointID;
}

bool cKinTree::IsRoot(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return !HasParent(joint_mat, joint_id);
}

bool cKinTree::IsJointActuated(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return !IsRoot(joint_mat, joint_id);
}

double cKinTree::GetTorqueLimit(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double torque_lim = joint_mat(joint_id, cKinTree::eJointDescTorqueLim);
	return torque_lim;
}

double cKinTree::GetForceLimit(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double force_lim = joint_mat(joint_id, cKinTree::eJointDescForceLim);
	return force_lim;
}

bool cKinTree::IsEndEffector(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double end_eff_val = joint_mat(joint_id, cKinTree::eJointDescIsEndEffector);
	return end_eff_val != 0;
}

tVector cKinTree::GetJointLimLow(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return tVector(joint_mat(joint_id, cKinTree::eJointDescLimLow0), 
					joint_mat(joint_id, cKinTree::eJointDescLimLow1), 
					joint_mat(joint_id, cKinTree::eJointDescLimLow2), 1);
}

tVector cKinTree::GetJointLimHigh(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return tVector(joint_mat(joint_id, cKinTree::eJointDescLimHigh0), 
					joint_mat(joint_id, cKinTree::eJointDescLimHigh1),
					joint_mat(joint_id, cKinTree::eJointDescLimHigh2), 0);
}

double cKinTree::GetJointDiffWeight(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return joint_mat(joint_id, cKinTree::eJointDescDiffWeight);
}

double cKinTree::CalcLinkLength(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	tVector attach_pt = GetAttachPt(joint_mat, joint_id);
	bool is_root = IsRoot(joint_mat, joint_id);
	double len = (is_root) ? 0 : attach_pt.norm();
	return len;
}

tVector cKinTree::GetAttachPt(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	tVector attach_pt = tVector(joint_mat(joint_id, cKinTree::eJointDescAttachX),
								joint_mat(joint_id, cKinTree::eJointDescAttachY),
								joint_mat(joint_id, cKinTree::eJointDescAttachZ), 0);
	return attach_pt;
}

tVector cKinTree::GetAttachTheta(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	tVector attach_theta = tVector(joint_mat(joint_id, cKinTree::eJointDescAttachThetaX),
									joint_mat(joint_id, cKinTree::eJointDescAttachThetaY),
									joint_mat(joint_id, cKinTree::eJointDescAttachThetaZ), 0);
	return attach_theta;
}

void cKinTree::CalcMaxSubChainLengths(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_lengths)
{
	int num_joints = static_cast<int>(joint_mat.rows());
	out_lengths = Eigen::VectorXd::Zero(num_joints);

	for (int j = num_joints - 1; j >= 0; --j)
	{
		int parent_id = GetParent(joint_mat, j);
		if (parent_id != gInvalidJointID)
		{
			double curr_val = out_lengths(j);
			double len = CalcLinkLength(joint_mat, j);
			double& parent_val = out_lengths(parent_id);

			if (parent_val < len + curr_val)
			{
				parent_val = len + curr_val;
			}
		}
	}
}

void cKinTree::CalcSubTreeMasses(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, Eigen::VectorXd& out_masses)
{
	int num_joints = static_cast<int>(joint_mat.rows());
	out_masses = Eigen::VectorXd::Zero(num_joints);

	for (int j = num_joints - 1; j >= 0; --j)
	{
		double& curr_val = out_masses(j);
		double mass = GetBodyMass(body_defs, j);
		curr_val += mass;

		int parent_id = GetParent(joint_mat, j);
		if (parent_id != gInvalidJointID)
		{
			double& parent_val = out_masses(parent_id);
			parent_val += curr_val;
		}
	}
}

bool cKinTree::ParseJointType(const std::string& type_str, eJointType& out_joint_type)
{
	for (int i = 0; i < eJointTypeMax; ++i)
	{
		const std::string& name = gJointTypeNames[i];
		if (type_str == name)
		{
			out_joint_type = static_cast<eJointType>(i);
			return true;
		}
	}
	printf("Unsupported joint type: %s\n", type_str.c_str());
	assert(false); // unsupported joint type
	return false;
}

void cKinTree::PostProcessJointMat(Eigen::MatrixXd& out_joint_mat)
{
	int num_joints = GetNumJoints(out_joint_mat);
	int offset = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		int curr_size = GetParamSize(out_joint_mat, j);
		out_joint_mat(j, eJointDescParamOffset) = offset;
		offset += curr_size;
	}
	int root_id = GetRoot(out_joint_mat);

	out_joint_mat(root_id, eJointDescAttachX) = 0;
	out_joint_mat(root_id, eJointDescAttachY) = 0;
	out_joint_mat(root_id, eJointDescAttachZ) = 0;
}

tMatrix cKinTree::BuildAttachTrans(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	// child to parent
	tVector attach_pt = GetAttachPt(joint_mat, joint_id);
	tVector attach_theta = GetAttachTheta(joint_mat, joint_id);
	tMatrix mat = cMathUtil::RotateMat(attach_theta);
	mat(0, 3) = attach_pt[0];
	mat(1, 3) = attach_pt[1];
	mat(2, 3) = attach_pt[2];
	return mat;
}

tMatrix cKinTree::ChildParentTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix mat;
	eJointType j_type = GetJointType(joint_mat, joint_id);
	bool is_root = IsRoot(joint_mat, joint_id);

	if (is_root)
	{
		mat = ChildParentTransRoot(joint_mat, state, joint_id);
	}
	else
	{
		switch (j_type)
		{
		case eJointTypeRevolute:
			mat = ChildParentTransRevolute(joint_mat, state, joint_id);
			break;
		case eJointTypePrismatic:
			mat = ChildParentTransPrismatic(joint_mat, state, joint_id);
			break;
		case eJointTypePlanar:
			mat = ChildParentTransPlanar(joint_mat, state, joint_id);
			break;
		case eJointTypeFixed:
			mat = ChildParentTransFixed(joint_mat, state, joint_id);
			break;
		case eJointTypeSpherical:
			mat = ChildParentTransSpherical(joint_mat, state, joint_id);
			break;
		default:
			break;
		}
	}
	
	return mat;
}

tMatrix cKinTree::ParentChildTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix mat = ChildParentTrans(joint_mat, state, joint_id);
	mat = cMathUtil::InvRigidMat(mat);
	return mat;
}

tMatrix cKinTree::JointWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix m = tMatrix::Identity();
	int curr_id = joint_id;
	assert(joint_id != gInvalidIdx); // invalid joint
	while (curr_id != gInvalidJointID)
	{
		tMatrix child_parent_mat = ChildParentTrans(joint_mat, state, curr_id);
		m = child_parent_mat * m;
		curr_id = GetParent(joint_mat, curr_id);
	}

	return m;
}

tMatrix cKinTree::WorldJointTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix m = JointWorldTrans(joint_mat, state, joint_id);
	m = cMathUtil::InvRigidMat(m);
	return m;
}

tMatrix cKinTree::BodyWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id)
{
	tMatrix body_trans = BodyJointTrans(body_defs, part_id);
	tMatrix joint_trans = JointWorldTrans(joint_mat, state, part_id);
	body_trans = joint_trans * body_trans;
	return body_trans;
}

tMatrix cKinTree::BodyJointTrans(const Eigen::MatrixXd& body_defs, int part_id)
{
	tVector attach_pt = GetBodyAttachPt(body_defs, part_id);
	tVector euler = GetBodyAttachTheta(body_defs, part_id);
	tVector com = GetBodyLocalCoM(body_defs, part_id);

	tMatrix rot = cMathUtil::RotateMat(euler);
	tMatrix trans = cMathUtil::TranslateMat(attach_pt + com);
	tMatrix m = trans * rot;
	return m;
}

cKinTree::tJointDesc cKinTree::BuildJointDesc(eJointType joint_type, int parent_id, const tVector& attach_pt)
{
	tJointDesc desc = BuildJointDesc();
	desc(eJointDescType) = static_cast<double>(joint_type);
	desc(eJointDescParent) = parent_id;
	desc(eJointDescAttachX) = attach_pt[0];
	desc(eJointDescAttachY) = attach_pt[1];
	desc(eJointDescAttachZ) = attach_pt[2];
	
	return desc;
}

cKinTree::tJointDesc cKinTree::BuildJointDesc()
{
	tJointDesc desc;
	desc(eJointDescType) = static_cast<double>(eJointTypeRevolute);
	desc(eJointDescParent) = gInvalidIdx;
	desc(eJointDescAttachX) = 0;
	desc(eJointDescAttachY) = 0;
	desc(eJointDescAttachZ) = 0;
	desc(eJointDescAttachThetaX) = 0;
	desc(eJointDescAttachThetaY) = 0;
	desc(eJointDescAttachThetaZ) = 0;
	desc(eJointDescLimLow0) = 1;
	desc(eJointDescLimLow1) = 1;
	desc(eJointDescLimLow2) = 1;
	desc(eJointDescLimHigh0) = 0;
	desc(eJointDescLimHigh1) = 0;
	desc(eJointDescLimHigh2) = 0;
	desc(eJointDescIsEndEffector) = 0;
	desc(eJointDescTorqueLim) = std::numeric_limits<double>::infinity();
	desc(eJointDescForceLim) = std::numeric_limits<double>::infinity();
	desc(eJointDescDiffWeight) = 1;
	desc(eJointDescParamOffset) = 0;

	return desc;
}

cKinTree::tBodyDef cKinTree::BuildBodyDef()
{
	tBodyDef def;
	def(eBodyParamShape) = static_cast<double>(cShape::eShapeNull);
	def(eBodyParamMass) = 0;
	def(eBodyParamColGroup) = gInvalidIdx;
	def(eBodyParamEnableFallContact) = 0;
	def(eBodyParamAttachX) = 0;
	def(eBodyParamAttachY) = 0;
	def(eBodyParamAttachZ) = 0;
	def(eBodyParamAttachThetaX) = 0;
	def(eBodyParamAttachThetaY) = 0;
	def(eBodyParamAttachThetaZ) = 0;
	def(eBodyParam0) = 0;
	def(eBodyParam1) = 0;
	def(eBodyParam2) = 0;
	def(eBodyColorR) = 0;
	def(eBodyColorG) = 0;
	def(eBodyColorB) = 0;
	def(eBodyColorA) = 1;
	return def;
}

cKinTree::tDrawShapeDef cKinTree::BuildDrawShapeDef()
{
	tDrawShapeDef def;
	def(eDrawShapeShape) = static_cast<double>(cShape::eShapeNull);
	def(eDrawShapeParentJoint) = gInvalidIdx;
	def(eDrawShapeAttachX) = 0;
	def(eDrawShapeAttachY) = 0;
	def(eDrawShapeAttachZ) = 0;
	def(eDrawShapeAttachThetaX) = 0;
	def(eDrawShapeAttachThetaY) = 0;
	def(eDrawShapeAttachThetaZ) = 0;
	def(eDrawShapeParam0) = 0;
	def(eDrawShapeParam1) = 0;
	def(eDrawShapeParam2) = 0;
	def(eDrawShapeColorR) = 0;
	def(eDrawShapeColorG) = 0;
	def(eDrawShapeColorB) = 0;
	def(eDrawShapeColorA) = 1;
	def(eDrawShapeMeshID) = gInvalidIdx;
	return def;
}

void cKinTree::BuildDefaultPose(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose)
{
	int num_dof = GetNumDof(joint_mat);
	out_pose = Eigen::VectorXd::Zero(num_dof);

	int num_joints = GetNumJoints(joint_mat);

	int root_id = GetRoot(joint_mat);
	Eigen::VectorXd root_pose;
	BuildDefaultPoseRoot(joint_mat, root_pose);
	SetJointParams(joint_mat, root_id, root_pose, out_pose);

	for (int j = 1; j < num_joints; ++j)
	{
		eJointType joint_type = GetJointType(joint_mat, j);
		Eigen::VectorXd joint_pose;
		switch (joint_type)
		{
		case eJointTypeRevolute:
			BuildDefaultPoseRevolute(joint_pose);
			break;
		case eJointTypePrismatic:
			BuildDefaultPosePrismatic(joint_pose);
			break;
		case eJointTypePlanar:
			BuildDefaultPosePlanar(joint_pose);
			break;
		case eJointTypeFixed:
			BuildDefaultPoseFixed(joint_pose);
			break;
		case eJointTypeSpherical:
			BuildDefaultPoseSpherical(joint_pose);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}

		SetJointParams(joint_mat, j, joint_pose, out_pose);
	}
}

void cKinTree::BuildDefaultVel(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_vel)
{
	int num_dof = GetNumDof(joint_mat);
	out_vel = Eigen::VectorXd::Zero(num_dof);

	int num_joints = GetNumJoints(joint_mat);

	int root_id = GetRoot(joint_mat);
	Eigen::VectorXd root_pose;
	BuildDefaultVelRoot(joint_mat, root_pose);
	SetJointParams(joint_mat, root_id, root_pose, out_vel);

	for (int j = 1; j < num_joints; ++j)
	{
		eJointType joint_type = GetJointType(joint_mat, j);
		Eigen::VectorXd joint_pose;
		switch (joint_type)
		{
		case eJointTypeRevolute:
			BuildDefaultVelRevolute(joint_pose);
			break;
		case eJointTypePrismatic:
			BuildDefaultVelPrismatic(joint_pose);
			break;
		case eJointTypePlanar:
			BuildDefaultVelPlanar(joint_pose);
			break;
		case eJointTypeFixed:
			BuildDefaultVelFixed(joint_pose);
			break;
		case eJointTypeSpherical:
			BuildDefaultVelSpherical(joint_pose);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}

		SetJointParams(joint_mat, j, joint_pose, out_vel);
	}
}

void cKinTree::CalcPoseDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, Eigen::VectorXd& out_diff)
{
	int num_joints = GetNumJoints(joint_mat);
	assert(out_diff.size() == pose0.size());
	assert(pose1.size() == pose0.size());

	out_diff.resize(pose1.size());
	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd curr_diff;
		CalcJointPoseDiff(joint_mat, j, pose0, pose1, curr_diff);

		int param_offset = GetParamOffset(joint_mat, j);
		int param_size = GetParamSize(joint_mat, j);
		out_diff.segment(param_offset, param_size) = curr_diff;
	}
}

tVector cKinTree::CalcRootPosDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tVector root_pos0 = GetRootPos(joint_mat, pose0);
	tVector root_pos1 = GetRootPos(joint_mat, pose1);
	return root_pos1 - root_pos0;
}

tQuaternion cKinTree::CalcRootRotDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tQuaternion root_rot0 = GetRootRot(joint_mat, pose0);
	tQuaternion root_rot1 = GetRootRot(joint_mat, pose1);
	return cMathUtil::QuatDiff(root_rot0, root_rot1);
}

double cKinTree::CalcPoseErr(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	bool is_root = IsRoot(joint_mat, joint_id);
	double err = 0;
	if (is_root)
	{
		double root_pos_err = CalcRootPosErr(joint_mat, pose0, pose1);
		double root_rot_err = CalcRootRotErr(joint_mat, pose0, pose1);
		err = root_pos_err + root_rot_err;
	}
	else
	{
		Eigen::VectorXd diff;
		CalcJointPoseDiff(joint_mat, joint_id, pose0, pose1, diff);

		eJointType joint_type = GetJointType(joint_mat, joint_id);
		switch (joint_type)
		{
		case eJointTypeSpherical:
		{
			tQuaternion dq = cMathUtil::VecToQuat(diff);
			double theta = cMathUtil::QuatTheta(dq);
			err = theta * theta;
			break;
		}
		default:
			err = diff.squaredNorm();
			break;
		}
	}
	return err;
}

double cKinTree::CalcRootPosErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tVector diff = CalcRootPosDiff(joint_mat, pose0, pose1);
	return diff.squaredNorm();
}

double cKinTree::CalcRootRotErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tQuaternion diff = CalcRootRotDiff(joint_mat, pose0, pose1);
	double theta = cMathUtil::QuatTheta(diff);
	return theta * theta;
}


void cKinTree::CalcVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1, Eigen::VectorXd& out_diff)
{
	int num_joints = GetNumJoints(joint_mat);
	assert(vel0.size() == vel1.size());

	out_diff.resize(vel0.size());
	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd curr_diff;
		CalcJointVelDiff(joint_mat, j, vel0, vel1, curr_diff);

		int param_offset = GetParamOffset(joint_mat, j);
		int param_size = GetParamSize(joint_mat, j);
		out_diff.segment(param_offset, param_size) = curr_diff;
	}
}

tVector cKinTree::CalcRootVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tVector root_vel0 = GetRootVel(joint_mat, pose0);
	tVector root_vel1 = GetRootVel(joint_mat, pose1);
	return root_vel1 - root_vel0;
}

tVector cKinTree::CalcRootAngVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1)
{
	tVector root_ang_vel0 = GetRootAngVel(joint_mat, pose0);
	tVector root_ang_vel1 = GetRootAngVel(joint_mat, pose1);
	return root_ang_vel1 - root_ang_vel0;
}

double cKinTree::CalcVelErr(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1)
{
	bool is_root = IsRoot(joint_mat, joint_id);
	double err = 0;
	if (is_root)
	{
		double root_vel_err = CalcRootVelErr(joint_mat, vel0, vel1);
		double root_ang_vel_err = CalcRootAngVelErr(joint_mat, vel0, vel1);
		err = root_vel_err + root_ang_vel_err;
	}
	else
	{
		Eigen::VectorXd diff;
		CalcJointVelDiff(joint_mat, joint_id, vel0, vel1, diff);
		err = diff.squaredNorm();
	}
	return err;
}

double cKinTree::CalcRootVelErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1)
{
	tVector diff = CalcRootVelDiff(joint_mat, vel0, vel1);
	return diff.squaredNorm();
}

double cKinTree::CalcRootAngVelErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1)
{
	tVector diff = CalcRootAngVelDiff(joint_mat, vel0, vel1);
	return diff.squaredNorm();
}


void cKinTree::CalcJointPoseDiff(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, Eigen::VectorXd& out_diff)
{
	int param_offset = GetParamOffset(joint_mat, joint_id);
	int param_size = GetParamSize(joint_mat, joint_id);
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);

	if (is_root)
	{
		tVector root_pos_diff = CalcRootPosDiff(joint_mat, pose0, pose1);
		tQuaternion root_rot_diff = CalcRootRotDiff(joint_mat, pose0, pose1);
		out_diff.resize(gRootDim);
		out_diff.segment(0, gPosDim) = root_pos_diff.segment(0, gPosDim);
		out_diff.segment(gPosDim, gRotDim) = cMathUtil::QuatToVec(root_rot_diff);
	}
	else
	{
		eJointType joint_type = GetJointType(joint_mat, joint_id);
		switch (joint_type)
		{
		case eJointTypeSpherical:
		{
			tQuaternion q0 = cMathUtil::VecToQuat(pose0.segment(param_offset, param_size));
			tQuaternion q1 = cMathUtil::VecToQuat(pose1.segment(param_offset, param_size));
			tQuaternion q_diff = cMathUtil::QuatDiff(q0, q1);
			out_diff = cMathUtil::QuatToVec(q_diff);
			break;
		}
		default:
			out_diff = pose1.segment(param_offset, param_size) - pose0.segment(param_offset, param_size);
			break;
		}
	}
}

void cKinTree::CalcJointVelDiff(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1, Eigen::VectorXd& out_diff)
{
	int param_offset = GetParamOffset(joint_mat, joint_id);
	int param_size = GetParamSize(joint_mat, joint_id);
	out_diff = vel1.segment(param_offset, param_size) - vel0.segment(param_offset, param_size);
}

void cKinTree::CalcVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, double dt, Eigen::VectorXd& out_vel)
{
	assert(pose0.size() == pose1.size());
	out_vel.resize(pose0.size());

	int num_joints = GetNumJoints(joint_mat);
	tVector root_pos0 = GetRootPos(joint_mat, pose0);
	tVector root_pos1 = GetRootPos(joint_mat, pose1);
	tVector root_vel = (root_pos1 - root_pos0) / dt;

	tQuaternion root_rot0 = GetRootRot(joint_mat, pose0);
	tQuaternion root_rot1 = GetRootRot(joint_mat, pose1);
	tVector root_rot_vel = cMathUtil::CalcQuaternionVelRel(root_rot0, root_rot1, dt);

	cKinTree::SetRootVel(joint_mat, root_vel, out_vel);
	cKinTree::SetRootAngVel(joint_mat, root_rot_vel, out_vel);

	for (int j = 1; j < num_joints; ++j)
	{
		int param_offset = GetParamOffset(joint_mat, j);
		int param_size = GetParamSize(joint_mat, j);
		eJointType joint_type = GetJointType(joint_mat, j);

		switch (joint_type)
		{
		case eJointTypeSpherical:
		{
			tQuaternion q0 = cMathUtil::VecToQuat(pose0.segment(param_offset, param_size));
			tQuaternion q1 = cMathUtil::VecToQuat(pose1.segment(param_offset, param_size));
			tVector rot_vel = cMathUtil::CalcQuaternionVelRel(q0, q1, dt);
			out_vel.segment(param_offset, param_size) = rot_vel.segment(0, param_size);
			break;
		}
		default:
			out_vel.segment(param_offset, param_size) = (pose1.segment(param_offset, param_size) - pose0.segment(param_offset, param_size)) / dt;
			break;
		}
	}
}

void cKinTree::PostProcessPose(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose)
{
	// mostly to normalize quaternions
	int num_joints = GetNumJoints(joint_mat);
	int root_id = GetRoot(joint_mat);
	int root_offset = GetParamOffset(joint_mat, root_id);
	out_pose.segment(root_offset + gPosDim, gRotDim).normalize();

	for (int j = 1; j < num_joints; ++j)
	{
		eJointType joint_type = GetJointType(joint_mat, j);
		if (joint_type == eJointTypeSpherical)
		{
			int offset = GetParamOffset(joint_mat, j);
			out_pose.segment(offset, gRotDim).normalize();
		}
	}
}

void cKinTree::LerpPoses(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, double lerp, Eigen::VectorXd& out_pose)
{
	int num_joints = GetNumJoints(joint_mat);
	int root_id = GetRoot(joint_mat);
	int root_offset = GetParamOffset(joint_mat, root_id);

	out_pose.resize(pose0.size());
	assert(pose0.size() == pose1.size());

	tVector root_pos0 = GetRootPos(joint_mat, pose0);
	tVector root_pos1 = GetRootPos(joint_mat, pose1);
	tVector root_pos_lerp = (1 - lerp) * root_pos0 + lerp * root_pos1;

	tQuaternion root_rot0 = GetRootRot(joint_mat, pose0);
	tQuaternion root_rot1 = GetRootRot(joint_mat, pose1);
	assert(std::abs(root_rot0.norm() - 1) < 0.000001);
	assert(std::abs(root_rot1.norm() - 1) < 0.000001);

	tQuaternion root_rot_lerp = root_rot0.slerp(lerp, root_rot1);
	root_rot_lerp.normalize();

	cKinTree::SetRootPos(joint_mat, root_pos_lerp, out_pose);
	cKinTree::SetRootRot(joint_mat, root_rot_lerp, out_pose);

	for (int j = 1; j < num_joints; ++j)
	{
		eJointType joint_type = GetJointType(joint_mat, j);
		int offset = GetParamOffset(joint_mat, j);
		int size = GetParamSize(joint_mat, j);
		if (joint_type == eJointTypeSpherical)
		{
			tQuaternion rot0 = cMathUtil::VecToQuat(pose0.segment(offset, size));
			tQuaternion rot1 = cMathUtil::VecToQuat(pose1.segment(offset, size));
			assert(std::abs(rot0.norm() - 1) < 0.000001);
			assert(std::abs(rot1.norm() - 1) < 0.000001);
			tQuaternion rot_lerp = rot0.slerp(lerp, rot1);
			out_pose.segment(offset, size) = cMathUtil::QuatToVec(rot_lerp).segment(0, size);
		}
		else
		{
			out_pose.segment(offset, size) = (1 - lerp) * pose0.segment(offset, size) + lerp * pose1.segment(offset, size);
		}
	}
}

void cKinTree::VelToPoseDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, Eigen::VectorXd& out_pose_diff)
{
	out_pose_diff = vel;

	int num_joints = GetNumJoints(joint_mat);
	int root_id = GetRoot(joint_mat);
	int root_offset = GetParamOffset(joint_mat, root_id);

	tVector root_rot_vel = GetRootAngVel(joint_mat, vel);
	root_rot_vel[3] = 0;
	tQuaternion root_rot = GetRootRot(joint_mat, pose);
	tMatrix root_diff_mat = cMathUtil::BuildQuaternionDiffMat(root_rot);
	tVector root_diff = root_diff_mat * root_rot_vel;
	tQuaternion root_quat = cMathUtil::VecToQuat(root_diff);

	cKinTree::SetRootRot(joint_mat, root_quat, out_pose_diff);

	for (int j = 1; j < num_joints; ++j)
	{
		eJointType joint_type = GetJointType(joint_mat, j);
		int offset = GetParamOffset(joint_mat, j);
		int size = GetParamSize(joint_mat, j);
		if (joint_type == eJointTypeSpherical)
		{
			tQuaternion rot = cMathUtil::VecToQuat(pose.segment(offset, size));
			tVector rot_vel = vel.segment(offset, size);
			rot_vel[3] = 0;
			tMatrix diff_mat = cMathUtil::BuildQuaternionDiffMat(rot);
			tVector q_diff = diff_mat * rot_vel;

			out_pose_diff.segment(offset, size) = q_diff;
		}
	}
}

double cKinTree::CalcHeading(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose)
{
	// heading is the direction of the root in the xz plane
	tVector ref_dir = tVector(1, 0, 0, 0);
	tQuaternion root_rot = cKinTree::GetRootRot(joint_mat, pose);
	printf("root_rot=%f,%f,%f,%f\n", root_rot.x(), root_rot.y(), root_rot.z(), root_rot.w());
	tVector rot_dir = cMathUtil::QuatRotVec(root_rot, ref_dir);
	double heading = std::atan2(-rot_dir[2], rot_dir[0]);
	return heading;
}

tQuaternion cKinTree::CalcHeadingRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose)
{
	double heading = CalcHeading(joint_mat, pose);
	return cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), heading);
}

tMatrix cKinTree::BuildHeadingTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose)
{
	double heading = CalcHeading(joint_mat, pose);
	tVector axis = tVector(0, 1, 0, 0);
	printf("heading=%f\n", heading);
	tMatrix mat = cMathUtil::RotateMat(axis, -heading);
	return mat;
}

tMatrix cKinTree::BuildOriginTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose)
{
	// world to origin transform
	// origin is the point right under the root of the character on the xz plane with x-axis
	// aligned along the character's heading
	tVector origin = GetRootPos(joint_mat, pose);
	printf("cKinTree::BuildOriginTrans: origin=%f,%f,%f\n", origin[0], origin[1], origin[2]);
	origin[1] = 0;
	tMatrix rot_mat = BuildHeadingTrans(joint_mat, pose);
	printf("headingMat = \n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f",
		rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2), rot_mat(0, 3),
		rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2), rot_mat(1, 3),
		rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2), rot_mat(2, 3),
		rot_mat(3, 0), rot_mat(3, 1), rot_mat(3, 2), rot_mat(3, 3));

	tMatrix trans_mat = cMathUtil::TranslateMat(-origin);
	tMatrix mat = rot_mat * trans_mat;
	printf("mat = \n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f",
		mat(0,0), mat(0, 1), mat(0, 2), mat(0, 3),
		mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
		mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3),
		mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3));
	return mat;
}

void cKinTree::NormalizePoseHeading(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose)
{
	Eigen::VectorXd vel= Eigen::VectorXd::Zero(0);
	NormalizePoseHeading(joint_mat, out_pose, vel);
}

void cKinTree::NormalizePoseHeading(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose, Eigen::VectorXd& out_vel)
{
	tVector root_pos = GetRootPos(joint_mat, out_pose);
	tQuaternion root_rot = GetRootRot(joint_mat, out_pose);

	root_pos[0] = 0;
	root_pos[2] = 0;

	double heading = CalcHeading(joint_mat, out_pose);
	tQuaternion heading_q = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), -heading);
	root_rot = heading_q * root_rot;

	SetRootPos(joint_mat, root_pos, out_pose);
	SetRootRot(joint_mat, root_rot, out_pose);

	if (out_vel.size() > 0)
	{
		tVector root_vel = GetRootVel(joint_mat, out_vel);
		tVector root_ang_vel = GetRootAngVel(joint_mat, out_vel);
		root_vel = cMathUtil::QuatRotVec(heading_q, root_vel);
		root_ang_vel = cMathUtil::QuatRotVec(heading_q, root_ang_vel);

		SetRootVel(joint_mat, root_vel, out_vel);
		SetRootAngVel(joint_mat, root_ang_vel, out_vel);
	}
}


tMatrix cKinTree::ChildParentTransRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector offset = GetRootPos(joint_mat, state);
	tQuaternion rot = GetRootRot(joint_mat, state);

	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix R = cMathUtil::RotateMat(rot);
	tMatrix T = cMathUtil::TranslateMat(offset);

	tMatrix mat = A * T * R;
	return mat;
}

tMatrix cKinTree::ChildParentTransRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	int param_offset = cKinTree::GetParamOffset(joint_mat, joint_id);
	double theta = state(param_offset);

	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), theta);
	
	tMatrix mat = A * R;
	return mat;
}

tMatrix cKinTree::ChildParentTransPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	int param_offset = cKinTree::GetParamOffset(joint_mat, joint_id);
	double theta = state(param_offset + 2);
	tVector offset = tVector::Zero();
	offset[0] = state(param_offset);
	offset[1] = state(param_offset + 1);

	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), theta);
	tMatrix T = cMathUtil::TranslateMat(offset);

	tMatrix mat = A * R * T;
	return mat;
}

tMatrix cKinTree::ChildParentTransFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix mat = A;
	return mat;
}

tMatrix cKinTree::ChildParentTransPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	int param_offset = cKinTree::GetParamOffset(joint_mat, joint_id);
	tVector offset = tVector::Zero();
	offset[0] = state(param_offset);

	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix T = cMathUtil::TranslateMat(offset);

	tMatrix mat = A * T;
	return mat;
}

tMatrix cKinTree::ChildParentTransSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	int param_offset = cKinTree::GetParamOffset(joint_mat, joint_id);
	int param_size = cKinTree::GetParamSize(joint_mat, joint_id);
	tQuaternion q = cMathUtil::VecToQuat(state.segment(param_offset, param_size));

	tMatrix A = BuildAttachTrans(joint_mat, joint_id);
	tMatrix R = cMathUtil::RotateMat(q);

	tMatrix mat = A * R;
	return mat;
}



void cKinTree::BuildDefaultPoseRoot(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose)
{
	int dim = gRootDim;
	out_pose = Eigen::VectorXd::Zero(dim);
	out_pose(gPosDim) = 1;
}

void cKinTree::BuildDefaultPoseRevolute(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeRevolute);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultPosePrismatic(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypePrismatic);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultPosePlanar(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypePlanar);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultPoseFixed(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeFixed);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultPoseSpherical(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeSpherical);
	out_pose = Eigen::VectorXd::Zero(dim);
	out_pose(0) = 1;
}


void cKinTree::BuildDefaultVelRoot(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose)
{
	int dim = gRootDim;
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultVelRevolute(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeRevolute);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultVelPrismatic(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypePrismatic);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultVelPlanar(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypePlanar);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultVelFixed(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeFixed);
	out_pose = Eigen::VectorXd::Zero(dim);
}

void cKinTree::BuildDefaultVelSpherical(Eigen::VectorXd& out_pose)
{
	int dim = GetJointParamSize(eJointTypeSpherical);
	out_pose = Eigen::VectorXd::Zero(dim);
}
