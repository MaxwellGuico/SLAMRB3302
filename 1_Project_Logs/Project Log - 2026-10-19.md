# Project Log [19 Oct]

Source: [Project Log [19 Oct]](https://app.notion.com/p/3feb182e8f04809d94b4cfbbfc65f324)

---

# Project Log



# What did i do

<ol>
<li>
Decreasing the inflation radius
</li>
<li>
Changed the initial pose python file
</li>
<li>
Increased from 8 to 20 initial pose estimates to increase initial correction
</li>
<li>
Pushed to github (Finally got it resolved, there is an ssh key to my github on the RB3302 Laptop)
</li>
<li></li>
</ol>





1. I think i should change back the standard deviation. Back to 0.2 from 0.1
2. I think we can TRY fusing angular z and absolute yaw
   1. Possible problem would be integration error accumulating from angular z
3. Tune these
   <ol>
   <li></li>
   </ol>

   amcl:

   ros\_\_parameters:

   update\_min\_d: 0.02

   update\_min\_a: 0.05

   sigma\_hit: 0.15

   [Image attachment: image.png — original media not included in this Markdown export]
