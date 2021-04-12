#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE
	if ((right_index - left_index) <= Threshold) {
		node->makeLeaf(left_index, right_index - left_index);
	}
	else {
		Vector max = node->getAABB().max;
		Vector min = node->getAABB().min;

		Vector diff = max - min;

		float aabb_dimension = MAX3(diff.x, diff.y, diff.z);

		int dimension = (diff.x == aabb_dimension) ? 0 : (diff.y == aabb_dimension) ? 1 : 2;

		Comparator cmp;
		cmp.dimension = dimension;

		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);
		
		//find split index
		int split_index = 0;

		for (int i = left_index; i < right_index; i++) {
			if (objects[i]->GetBoundingBox().centroid().getAxisValue(dimension) < min.getAxisValue(dimension) + aabb_dimension / 2) {
				split_index++;
			}
			else {
				split_index++;
				break;
			}
		}
		if (split_index == 0 || split_index == right_index - left_index) {
			split_index = round((right_index - left_index) / 2);
		}

		BVHNode* left_node = new BVHNode();
		BVHNode* right_node = new BVHNode();

		//bbox of left node
		Vector min_left = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max_left = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB bbox_left = AABB(min_left, max_left);

		for (int i = left_index; i < split_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			bbox_left.extend(bbox);
		}

		bbox_left.min.x -= EPSILON; bbox_left.min.y -= EPSILON; bbox_left.min.z -= EPSILON;
		bbox_left.max.x += EPSILON; bbox_left.max.y += EPSILON; bbox_left.max.z += EPSILON;
		left_node->setAABB(bbox_left);

		//bbox of right node
		Vector min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB bbox_right = AABB(min_right, max_right);

		for (int i = split_index; i < right_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			bbox_right.extend(bbox);
		}

		bbox_right.min.x -= EPSILON; bbox_right.min.y -= EPSILON; bbox_right.min.z -= EPSILON;
		bbox_right.max.x += EPSILON; bbox_right.max.y += EPSILON; bbox_right.max.z += EPSILON;
		right_node->setAABB(bbox_right);

		//Initiate current node as an interior node
		node->makeNode(nodes.size());
		nodes.push_back(left_node);
		nodes.push_back(right_node);

		build_recursive(left_index, split_index, left_node);
		build_recursive(split_index, right_index, right_node);
	}

		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	    // node.index can have a index of objects vector or a index of nodes vector
			
		
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];
			
			//PUT YOUR CODE HERE
			return false;
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			//PUT YOUR CODE HERE
			return false;
	}		