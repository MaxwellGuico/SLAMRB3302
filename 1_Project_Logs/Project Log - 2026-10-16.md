# Project Log [16 Oct]

Source: [Project Log [16 Oct]](https://app.notion.com/p/3fbb182e8f0480888372cc2929bf2086)

---

# What did i do

1. Made a map based on project pdf and mapped on \[12 Oct\]
2. Changed amcl parameters
   ```javascript
       alpha1: 0.3 # Change when Rotation Estimate is noisy # changed this
       alpha2: 0.20 # Change when straight motion introduces heading uncertainty
       alpha3: 0.20 # Change Translation odom is noisy
       alpha4: 0.20 # Change when turns introduce translation uncertainty
       alpha5: 0.10
           # Update more frequently in the small 0.60 m cells.
       update_min_d: 0.05
       update_min_a: 0.01 # og 0.05 # Heading Corrections appear to frequently # need to investigate Oct 16
       resample_interval: 1

       min_particles: 300
       max_particles: 1000
       max_beams: 60 # 120 og

       laser_model_type: likelihood_field
       laser_likelihood_max_dist: 0.50
       laser_min_range: -1.0
       laser_max_range: -1.0
       sigma_hit: 0.20 # need to investigate Oct 16
       z_hit: 0.90 # trust the map more
       z_rand: 0.10
   ```

   3. Changed inflation radius
      0.15
   4. changed dist\_centre
      ```javascript
       def _publish_odom(self):
              now = self.get_clock().now()
              with self.feedback_lock:
                  # Changed dist_centre on Oct 16
                  dist_centre = 0.5 * (float(self.dist_L) + float(self.dist_R)) # float(self.dist_centre) 
      ```

# Whats next

## Trouble shoot

1. When initial pose given, its not aligning to the map
   [Image attachment: image.png — original media not included in this Markdown export]
2. when doing use\_rotate\_to\_heading: true it rotates slowly on the spot and doesnt move in translation
3. When i re build the robot\_ws because i changed the dist\_centre, when i run the usual commands, when i rotate the robot using teleop, the robot’s heading doesnt update. Not sure if the error is because of the build or what. Plus the changes i made were simple and hmmm not sure anymore.

## Thoughts

<ol>
<li></li>
</ol>
