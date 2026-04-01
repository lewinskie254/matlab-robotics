function particles_SF = Fusion_state(particles_robot1, particles_robot2, particles_SF, index_fov)
global num_landmarks num_particles
for l = 1:num_landmarks     
    for p = 1:num_particles                   
        if index_fov(3,l) % seen by both robots
            a = particles_robot1(p).landmarks(l).pos;
            b = particles_robot2(p).landmarks(l).pos;
            A = particles_robot1(p).landmarks(l).P;
            B = particles_robot2(p).landmarks(l).P;
            
            particles_SF(p).landmarks(l).pos = a+A/(A+B)*(b-a);
            particles_SF(p).landmarks(l).P   = A-A/(A+B)*A';
            continue;
        end
        if index_fov(1,l) % seen by robot1 only
            particles_SF(p).landmarks(l).pos = particles_robot1(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_robot1(p).landmarks(l).P;
        end
        if index_fov(2,l) % seen by robot2 only
            particles_SF(p).landmarks(l).pos = particles_robot2(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_robot2(p).landmarks(l).P;
        end
    end    
end