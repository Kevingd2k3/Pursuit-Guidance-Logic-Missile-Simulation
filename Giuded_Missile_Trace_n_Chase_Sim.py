# Based on / Inspired by: [https://github.com/Juan-David-Perez-Programming/trace-and-chase-missile-guidance]
# Adapted by Kevin G D to include [User Controls, STL Visualization, Splash Logic]

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh  # pip install numpy-stl
import time

# CLASSIFICATION FOR STL MODELS

class SimulatableObject:
    def __init__(self, stl_path, scale=1.0, color='blue', alpha=1.0, simplify_factor=2, fix_orientation=False):
        try:
            self.mesh_data = mesh.Mesh.from_file(stl_path)
            
            # 1. SLICE (Preserve 50% Detail @ [1:2]) to reduce cpmputational time while simulating
            vectors = self.mesh_data.vectors[::simplify_factor]
            
            # 2. ORIENTATION CORRECTION (Based on SolidWorks Check)
            # Input: Nose = -Z, Up = +Y (Bottom parallel to ZX)
            # Output: Nose = +X, Up = +Z (Standard Simulation Frame)
            if fix_orientation:
                correction_matrix = np.array([
                    [ 0,  0, -1],
                    [-1,  0,  0],
                    [ 0,  1,  0]
                ])
                
                # Apply rotation to every point
                shape = vectors.shape
                vectors = vectors.reshape(-1, 3) # Flatten
                vectors = vectors.dot(correction_matrix.T) # Rotate
                vectors = vectors.reshape(shape) # Reshape back

            # 3. FORCE CENTER 
            all_points = vectors.reshape(-1, 3)
            min_vals = np.min(all_points, axis=0)
            max_vals = np.max(all_points, axis=0)
            center = (max_vals + min_vals) / 2
            centered_vectors = vectors - center

            # 4. FORCE NORMALIZE (Fix Size Issues)
            max_dim = np.max(max_vals - min_vals)
            if max_dim == 0: max_dim = 1 
            self.base_vectors = (centered_vectors / max_dim)
            
        except Exception as e:
            print(f"Error loading {stl_path}: {e}")
            self.base_vectors = None

        self.scale = scale
        self.color = color
        self.alpha = alpha
        self.collection = None

    def get_rotation_matrix(self, velocity):
        """Calculates rotation matrix to align object +X axis with velocity."""
        vx, vy, vz = velocity
        if np.linalg.norm(velocity) == 0: return np.eye(3)
        
        # Yaw (Z-axis rotation)
        yaw = np.arctan2(vy, vx)
        
        # Pitch (Y-axis rotation)
        horiz_dist = np.sqrt(vx**2 + vy**2)
        pitch = np.arctan2(vz, horiz_dist)
        
        # Aerospace Rotation Matrices (Yaw then Pitch)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,            0,           1]
        ])
        
        # Note: Negative pitch is usually required to map math angles to visual angles
        Ry = np.array([
            [np.cos(-pitch), 0, np.sin(-pitch)],
            [0,              1, 0],
            [-np.sin(-pitch), 0, np.cos(-pitch)]
        ])
        
        return Rz @ Ry

    def update_plot(self, position, velocity, ax):
        if self.base_vectors is None: return

        if np.linalg.norm(velocity) < 0.1:
            R = np.eye(3)
        else:
            R = self.get_rotation_matrix(velocity)

        # 1. Rotate
        rotated_vectors = np.dot(self.base_vectors.reshape(-1, 3), R.T).reshape(-1, 3, 3)
        
        # 2. Scale & Translate
        transformed_vectors = (rotated_vectors * self.scale) + position

        if self.collection is None:
            self.collection = Poly3DCollection(transformed_vectors, alpha=self.alpha, facecolors=self.color)
            ax.add_collection3d(self.collection)
        else:
            self.collection.set_verts(transformed_vectors)
            self.collection.set_zsort('max')

# Variables

Straight_time = 25      
curve_time = 25         
Straight_time2 = 25     
targ_vel = 750          
miss_vel = 800          
turn_angle = -np.pi*4/3   
tmax = 75
dt = 0.05               
animation_interval = 20 
yz_angle = -np.pi/12
missile_start_loc = np.array([13000, 12000, 0])
aircraft_start_loc = np.array([0, 0, 12000]) 
missile_launch_time = 0 
kill_dist = 50          
climb_rate_curve = -0.001

# VISUAL SCALING 
SCALE_F15 = 6000.0        
SCALE_MISSILE = 3000.0   

# TRAJECTORY GENERATION

curve_start_x = None; curve_start_y = None; curve_start_z = None; curve_initialized = False
straight2_start_x = None; straight2_start_y = None; straight2_start_z = None; straight2_initialized = False
radius = (targ_vel * curve_time) / turn_angle 
center_x = None; center_y = None; center_z = None

def target_location(t, target_states):
    global curve_start_x, curve_start_y, curve_start_z, curve_initialized
    global straight2_start_x, straight2_start_y, straight2_start_z, straight2_initialized
    global center_x, center_y, center_z
    
    if 0 <= t <= Straight_time:
        x = aircraft_start_loc[0] + targ_vel * t
        y = aircraft_start_loc[1]
        z = aircraft_start_loc[2]
        
    elif Straight_time < t <= Straight_time + curve_time:
        tc = t - Straight_time
        if not curve_initialized:
            if len(target_states) > 0:
                curve_start_x = target_states[-1, 0]; curve_start_y = target_states[-1, 1]; curve_start_z = target_states[-1, 2]
            else:
                curve_start_x = aircraft_start_loc[0] + targ_vel * Straight_time
                curve_start_y = aircraft_start_loc[1]
                curve_start_z = aircraft_start_loc[2]
            center_x = curve_start_x; center_y = curve_start_y + radius * np.cos(yz_angle); center_z = curve_start_z + radius * np.sin(yz_angle)
            curve_initialized = True
        
        angle = tc * turn_angle / curve_time 
        arc_angle = -np.pi/2 + angle
        x = center_x + radius * np.cos(arc_angle)
        y = center_y + radius * np.sin(arc_angle) * np.cos(yz_angle) + np.cos(yz_angle+np.pi/2) * targ_vel**2 * (1-np.cos(np.pi*tc/curve_time)) * climb_rate_curve
        z = center_z + radius * np.sin(arc_angle) * np.sin(yz_angle) + np.sin(yz_angle+np.pi/2) * targ_vel**2 * (1-np.cos(np.pi*tc/curve_time)) * climb_rate_curve
        
    elif Straight_time + curve_time < t <= Straight_time + curve_time + Straight_time2:
        if not straight2_initialized:
            if len(target_states) > 0:
                straight2_start_x = target_states[-1, 0]; straight2_start_y = target_states[-1, 1]; straight2_start_z = target_states[-1, 2]
            else:
                straight2_start_x = curve_start_x; straight2_start_y = curve_start_y; straight2_start_z = curve_start_z
            straight2_initialized = True
        ts = t - (Straight_time + curve_time)
        dx = np.cos(turn_angle)
        dy = np.sin(turn_angle) * np.cos(yz_angle)
        dz = np.sin(turn_angle) * np.sin(yz_angle)
        x = straight2_start_x + targ_vel * ts * dx
        y = straight2_start_y + targ_vel * ts * dy
        z = straight2_start_z + targ_vel * ts * dz
    else:
        # Fallback logic for out of time bounds
        dx = np.cos(turn_angle)
        dy = np.sin(turn_angle) * np.cos(yz_angle)
        dz = np.sin(turn_angle) * np.sin(yz_angle)
        x = straight2_start_x + targ_vel * Straight_time2 * dx
        y = straight2_start_y + targ_vel * Straight_time2 * dy
        z = straight2_start_z + targ_vel * Straight_time2 * dz
    return np.array([x, y, z])

# Generate Arrays
curve_initialized = False; straight2_initialized = False
times = np.arange(0, tmax, dt)
n_points = len(times)
target_states = np.zeros((n_points, 3))
for i in range(n_points):
    target_states[i] = target_location(times[i], target_states[:i])

missile_states = np.zeros((n_points, 3))
missile_states[0] = missile_start_loc
missile_launched = False; intercept_time = None; intercepted = False 

for i in range(1, n_points):
    t = times[i]
    if t >= missile_launch_time and not missile_launched: missile_launched = True
    
    if missile_launched:
        if intercepted:
            missile_states[i] = missile_states[i-1]
            continue
        direction = target_states[i] - missile_states[i-1]
        distance = np.linalg.norm(direction)
        if distance < kill_dist and intercept_time is None:
            intercept_time = t; intercepted = True 
            missile_states[i] = missile_states[i-1] 
            continue
        if distance > 0:
            missile_states[i] = missile_states[i-1] + (direction / distance) * miss_vel * dt
        else:
            missile_states[i] = missile_states[i-1]
    else:
        missile_states[i] = missile_start_loc

# ... (inside the pre-calculation loop)
missile_states = np.zeros((n_points, 3))
missile_states[0] = missile_start_loc
missile_launched = False; intercept_time = None; intercepted = False 
intercept_index = None  # <--- NEW VARIABLE

for i in range(1, n_points):
    t = times[i]
    if t >= missile_launch_time and not missile_launched: missile_launched = True
    
    if missile_launched:
        if intercepted:
            missile_states[i] = missile_states[i-1]
            continue
        direction = target_states[i] - missile_states[i-1]
        distance = np.linalg.norm(direction)
        
        # MODIFIED CHECK 
        if distance < kill_dist and intercept_time is None:
            intercept_time = t
            intercepted = True
            intercept_index = i  # <--- SAVE THE FRAME INDEX
            missile_states[i] = missile_states[i-1] 
            continue
        
        if distance > 0:
            missile_states[i] = missile_states[i-1] + (direction / distance) * miss_vel * dt
        else:
            missile_states[i] = missile_states[i-1]
    else:
        missile_states[i] = missile_start_loc

# INITIALIZE STLS

try:
    
    # F15
    f15_model = SimulatableObject('F15.STL', scale=SCALE_F15, color='silver', simplify_factor=2, fix_orientation=True)

    # Missile (Also pointed -Z, so we apply the same correction)
    missile_model = SimulatableObject('Missile.stl', scale=SCALE_MISSILE, color='red', simplify_factor=2, fix_orientation=True)
    
    has_stls = True
    print("STLs loaded successfully with SolidWorks Orientation Correction.")
except Exception as e:
    print(f"Error loading STLs: {e}")
    has_stls = False

# PLOTTING

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

all_points = np.vstack([target_states, missile_states])
plot_radius = max(np.ptp(all_points[:, 0]), np.ptp(all_points[:, 1]), np.ptp(all_points[:, 2])) / 2 * 1.1
centers = np.mean(all_points, axis=0)

ax.set_xlim(centers[0] - plot_radius, centers[0] + plot_radius)
ax.set_ylim(centers[1] - plot_radius, centers[1] + plot_radius)
ax.set_zlim(centers[2] - plot_radius, centers[2] + plot_radius)
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_box_aspect([1, 1, 1])
ax.set_title('3D Pursuit Simulation')
ax.grid(True)
ax.view_init(elev=30, azim=-60)

target_trail, = ax.plot([], [], [], 'b-', linewidth=1, alpha=0.3, label='F-15 Path')
missile_trail, = ax.plot([], [], [], 'r-', linewidth=1, alpha=0.3, label='Missile Path')

if not has_stls:
    target_point, = ax.plot([], [], [], 'bo'); missile_point, = ax.plot([], [], [], 'ro')

time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)
dist_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes)

def init():
    return target_trail, missile_trail

def update(frame):
    # Reset visibility if animation loops back to start
    if frame == 0:
        if has_stls and f15_model.collection: f15_model.collection.set_alpha(f15_model.alpha)
        if has_stls and missile_model.collection: missile_model.collection.set_alpha(missile_model.alpha)

    current_frame = frame
    
    # If we have intercepted and passed the moment...
    if intercepted and intercept_index is not None and frame >= intercept_index:
        # 1. Clamp frame to the intercept moment (stops path from growing)
        current_frame = intercept_index 
        
        # 2. Make STLs Disappear (Set transparency to 0)
        if has_stls:
            if f15_model.collection: f15_model.collection.set_alpha(0)
            if missile_model.collection: missile_model.collection.set_alpha(0)
    # -----------------------------------

    # Update Trails (using current_frame, which might be frozen)
    target_trail.set_data(target_states[:current_frame+1, 0], target_states[:current_frame+1, 1])
    target_trail.set_3d_properties(target_states[:current_frame+1, 2])
    missile_trail.set_data(missile_states[:current_frame+1, 0], missile_states[:current_frame+1, 1])
    missile_trail.set_3d_properties(missile_states[:current_frame+1, 2])
    
    # Calculate velocities for rotation
    if current_frame < n_points - 1:
        vel_targ = target_states[current_frame+1] - target_states[current_frame]
        vel_miss = missile_states[current_frame+1] - missile_states[current_frame]
    else:
        vel_targ = target_states[current_frame] - target_states[current_frame-1]
        vel_miss = missile_states[current_frame] - missile_states[current_frame-1]

    # Update STLs (Only if we haven't hidden them yet)
    if has_stls:
        # Only update position if we are NOT at/past the intercept (otherwise they are hidden anyway)
        if not (intercepted and frame >= intercept_index):
            f15_model.update_plot(target_states[current_frame], vel_targ, ax)
            missile_model.update_plot(missile_states[current_frame], vel_miss, ax)
    else:
        target_point.set_data([target_states[current_frame, 0]], [target_states[current_frame, 1]])
        target_point.set_3d_properties([target_states[current_frame, 2]])
        missile_point.set_data([missile_states[current_frame, 0]], [missile_states[current_frame, 1]])
        missile_point.set_3d_properties([missile_states[current_frame, 2]])

    current_dist = np.linalg.norm(target_states[current_frame] - missile_states[current_frame])
    time_text.set_text(f'Time: {times[current_frame]:.1f}s')
    
    if intercepted and frame >= intercept_index:
        dist_text.set_text(f'SPLASH! (Sep: {current_dist:.0f}m)')
    else:
        dist_text.set_text(f'Sep: {current_dist:.0f}m')
        
    return target_trail, missile_trail

frame_skip = 3 
anim = FuncAnimation(fig, update, frames=range(0, n_points, frame_skip), init_func=init, blit=False, interval=animation_interval)
plt.show()

    # simplify_factor = 2 only 50 % of mesh details / triangles are preserved 
    # you may change this settings depending on your machine capabilities
    # frame_skip = 3 also to save computational time and improve speed