## Why I Chose This Issue

I chose issue #943 "Optimize receive_msg ioctl to copy only actual received entries" because 
I am interested in further improving my skills in C++ and Python! This aligns with my interest in expanding my experience while also allowing me to learn as it is a good first issue to start with! It also doesn't have many but also has some comments which makes it seem like a good issue to start on.

I'm interested in this because:
1. I understand the fundamentals used in this project.
2. I have read the description in github and it does not seem complex to fix this issue.
3. The reviewers have also helped out a lot when students are pushing their commit's.
4. I want to start easy on a hands on issue like this one

From reading the issue thread, I understand the current problem is that rather than copying the whole array, we should only copy the entires that were recieved with the ret_entry_num. This will reduce the # of copies while also getting more copies later on rather than copying the entire array in the userspace.

I also left a comment showing interest, by saying "Hi! I am a student in AI 301 and am willing to take this issue on! I want to confirm if this issue is still open!"
