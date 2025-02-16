#ifndef QUADTREE_H
#define QUADTREE_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer to the Quadtree structure
typedef struct Quadtree Quadtree;

// Initialize a new quadtree with given center coordinates and side length
Quadtree* quadtree_init(float side_len, float center_x, float center_y);

// Deallocate and destroy the quadtree
void quadtree_deinit(Quadtree* tree);

// Reset the quadtree with new parameters
void quadtree_reset(Quadtree* tree, float side_len, float center_x, float center_y);

// Set the threshold for large body approximation
void quadtree_set_threshold(Quadtree* tree, float threshold);

// Set the force equation parameters
void quadtree_set_force_params(Quadtree* tree, float c, float r);

// Insert a point with mass into the quadtree
// Returns 0 on success, -1 on failure
int quadtree_insert(Quadtree* tree, float position_x, float position_y, float mass);

// Remove a point with mass from the quadtree
// Returns 0 on success, -1 on failure
int quadtree_remove(Quadtree* tree, float position_x, float position_y, float mass);

// Get the total mass stored in the quadtree
float quadtree_get_total_mass(Quadtree* tree);

// Get the force exerted on a body
void quadtree_get_force(Quadtree* tree, float position_x, float position_y, float mass, float* result_x, float* result_y);

// Get the nearest body (inclusive)
void quadtree_get_nearest_body_inclusive(Quadtree* tree, float position_x, float position_y, float* result_x, float* result_y, float* result_mass);

// Get the nearest body (exclusive)
void quadtree_get_nearest_body_exclusive(Quadtree* tree, float position_x, float position_y, float* result_x, float* result_y, float* result_mass);

#ifdef __cplusplus
}
#endif

#endif // QUADTREE_H
