# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   1. Pass a vector into a function defined as "Vector2D normalize_vector(Vector2D v) function, then return the normalize vector object as a copy.
   2. Pass a vector into a normalize_vector(Vector2D & v) function by reference, and normalize the vector in place.
   3. Overload the operator "/" to allow Vector2D object to be divided by a double. Create a function defined as "double vector_magnitude(Vector2D v)" which calculates and returns the magnitude of the vector passed as a parameter. Divide the original vector by the magnitude, and create a new normalized vector.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   1. This method I like because it creates a copy of the input vector, normalizes it, and then returns it. It's simple and easy to understand, although there could be more optimization probably. This aligns with F.1 from the C++ core guidelines, because the magnitude of the vector is used directly, and possibly always, for normalizing the vector, so that math operation can just go inside the function.
   2. This method is nice because it is faster in that it doesn't have to create a copy of the Vector2D object. A big con is that you modify the original vector, so you can't perform actions on it later unless you make a copy before normalizing the vector, which is awkward.
   3. This method aligns with C++ core guidelines F.2 and perhaps F.3 more so that the other two. It performs a smaller piece of the core functionality necessary to normalize the vector, while the entire operation can still be completed in one line.

   - Which of the methods would you implement and why?  
   I would implement the first method, and I did. While it's tempting to modify the Vector2D object in place using a reference as described in the second method, I also wanted to leave in the possibility that I would need the actual vector being normalized later, and wanted to avoid altering it forever. I also felt like it was unlikely I would need to create the magnitute of a vector in the class's implementation, so I would just be creating extra work for myself if I went with method 3.

2. What is the difference between a class and a struct in C++?  
Classes and structs are only different in that members are by default public in structs, and by default private in classes.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?  
I think that Transform2D is a class because of C.9: Minimize exposure of members. Only allowing access to the individual elements of the transformation matrix through class member functions limits the possible ways in which users can modify or interact with the data, which makes debugging easier. I believe that Vector2D is a struct because there isn't an invariant, meaning we don't need to establish any of the member variables using a constructor, and the data can vary independently. This is from C++ core guidelines C.2.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
You should declare single-argument constructors explicit to prevent unexpected implicit conversions. C++ core guideline C.46.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   The reason inv() is declared const is because it's undesireable to modify the Transform2D object it's being called on. To prohibit this behavior in the implementation of this function, the const keyword is used. There's no need to change the value of the original Transform2D object, Con.1.
   Conversely, the desired behavior of the *= operator is to modify the Transform2D object it's being called on, so this operator overloading function shouldn't be declared with the const keyword.