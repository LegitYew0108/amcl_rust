#[allow(unused)]
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info
};

struct ParticleVector{
    pub x:f64,
    pub y:f64,
    pub yaw:f64
}

struct Particle{
    pose: ParticleVector,
    likelihood: usize
}

impl Particle{
    pub fn calc_likelihood(&mut self, map: nav_msgs::msg::OccupancyGrid, sensor_data: sensor_msgs::msg::LaserScan, lidar_pose: ParticleVector,sigma_hit:f64, z_hit: f64, z_rand: f64){
        let added_pose = Particle::coord_add(&lidar_pose,&self.pose);
        let z_hit_denom:f64 = 2_f64 * sigma_hit * sigma_hit;
        let z_rand_mult:f64 = 1_f64 / (sensor_data.range_max as f64);
        let mut p: f64 = 1_f64;
        let range_count: usize = ((sensor_data.angle_max - sensor_data.angle_min)/sensor_data.angle_increment)as usize;
        let range_slice: &[f32] = sensor_data.ranges.as_slice();
        let mut angle_now = sensor_data.angle_min;
        let mut hit:ParticleVector = ParticleVector{x:0_f64,y:0_f64,yaw:0_f64};

        for i in 0..range_count{
            let mut pz:f64 = 0_f64;
            let mut z:f64 = 0_f64;
            hit.x = added_pose.x + (range_slice[i] as f64)*f64::cos(added_pose.yaw+(angle_now as f64));
            hit.y = added_pose.y + (range_slice[i] as f64)*f64::cos(added_pose.yaw+(angle_now as f64));
            angle_now += sensor_data.angle_increment;
            //Gaussian model
            pz += z_hit * f64::exp(-(z*z)/z_hit_denom);
            //random measurement
            pz += z_rand * z_rand_mult;
            p += pz * pz * pz;
        }
    }

    //bの座標系にaの座標系を加算
    fn coord_add(a: &ParticleVector, b: &ParticleVector) -> ParticleVector{
        let mut c:ParticleVector = ParticleVector{x:0_f64,y:0_f64,yaw:0_f64};
        c.x = b.x + a.x * f64::cos(b.yaw) - a.y * f64::sin(b.yaw);
        c.y = b.y + a.x * f64::sin(b.yaw) + a.y * f64::cos(b.yaw);
        c.yaw = a.yaw + b.yaw;
        c.yaw = f64::atan2(f64::sin(c.yaw) ,f64::cos(c.yaw));
        c
    }
}
