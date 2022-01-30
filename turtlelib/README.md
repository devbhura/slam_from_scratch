# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
    One method would be to create a standalone function
    A second method would be to create a function inside the vector2D struct
    A third would be to create a function in Transform2D class to do so. 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   This method would work well as it could utilized by other functions or class. However, since it is not private it is not a desirable design as private data could be hidden. 

   The second method would make it easy to use for a Vector2D however it would be specific to only variables of that structure type.

   This method makes it accessible to all members of a class but decreases ease of usability. 

   - Which of the methods would you implement and why?
   I would implement method 2, as normalize method can be easily used for any vector and its implementation would be specific to vector (easy to design) rather than something designed more universally.  

2. What is the difference between a class and a struct in C++?
In struct, data members can vary independently, while in class, an invariant is created. 
Struct members are public by default, class members are private by default. 

3. Why is Vector2D a struct and Transform2D Class (refer to at least 2 specific C++ core guidelines in your answer)?
Vector2D members are meant to be public so it a struct. Transform2D has private members as it is a class. Transform2D has invariants that allows member functions to be called. Struct does not 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
To avoid implicit conversion operators, as they are hard to debug and introduce unknowns that are hard to pinpoint. 

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
const makes it a constant, and in operator *=, since we change the observable state of the variable, we should not make it const. In the case for .inv(), since it returns a Vector2D type, one should make it const to avoid any race conditions.  

# Sample Run of frame_main

Enter transform T_{a,b}:
deg: 90 x: 0 y: 1

Enter transform T_{b,c}:
deg: 90 x: 1 y: 0

T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2

Enter vector v_b:
1 1 

v_bhat:[0.707107, 0.707107]
v_a:[-1, 2]
v_b:[1, 1]
v_c:[1, 1.11022e-16]

Enter Twist V_b:
1 1 1

V_a:[1, 0, 1]
V_b:[1, 1, 1]
V_c:[1, 2, -1]