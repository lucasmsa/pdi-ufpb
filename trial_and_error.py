#### Agora, será necessário passar de pontos principais das imagens para pixels. Para tanto, será utilizado _triangular mesh_. Com os principais pontos faciais de ambas as imagens, podemos definir que o mesmo _triangular mesh_ será aplicado nos dois grupos de pontos. Que retornará as correspondências triângulo por triângulo entre os dois grupos de pontos. O uso de triângulos é justificado pela facilidade na junção entre diferentes _triangular meshes_

#### Dividindo a imagem em triângulos, fazendo a média ponderada dos valores das cores (_cross-dissolve_), achando os triângulos intermediários das imagens

#### Trabalharemos com uma coordenada homogênea associada aos pontos (x, y) das coordenadas orginais. O que irá permitir o processo de translação, com a coordenada homogênea = 1
# $\begin{bmatrix}
#    x' \\ y' \\ 1   
# \end{bmatrix}  = \begin{bmatrix}
#    a & b & c \\
#    d & e & f \\
#    0 & 0 & 1
# \end{bmatrix} 
# \begin{bmatrix}
#    x \\ y \\ 1
# \end{bmatrix}$
# def draw_delaunay(size, subdivision, pixel_dictionary):
#     delaunay_array = []
#     triangles_list = subdivision.getTriangleList()
#     rectangle = (0, 0, correspondance_points['size'][0], correspondance_points['size'][1])

#     for triangle in triangles_list:
#         point_1, point_2, point_3 = (int(triangle[0]), int(triangle[1])),\
#                                     (int(triangle[2]), int(triangle[3])),\
#                                     (int(triangle[4]), int(triangle[5]))
        
#         rectangle_inside_borders = rectangle_contains(rectangle, point_1) and \
#                                    rectangle_contains(rectangle, point_2) and \
#                                    rectangle_contains(rectangle, point_3)
        
#         if rectangle_inside_borders:
#             delaunay_array.append((pixel_dictionary[point_1], pixel_dictionary[point_2], pixel_dictionary[point_3]))

#     return delaunay_array  

# def rectangle_contains(rectangle, point):
#     rectangle_out_of_bounds = point[0] < rectangle[0] or point[1] < rectangle[1] \
#                                or point[0] > rectangle[2] or point[1] > rectangle[3]

#     if rectangle_out_of_bounds: return False

#     return True

# def delaunay_triangulation(correspondance_points):
#     rectangle = (0, 0, correspondance_points['size'][0], correspondance_points['size'][1])
#     subdivision = cv2.Subdiv2D(rectangle)
#     correspondance_list = correspondance_points['correspondances'].tolist()
#     image_points = [(int(pixel[0]), int(pixel[1])) for pixel in correspondance_list]
#     pixel_dictionary = {pixel[0]:pixel[1] for pixel in list(zip(image_points, range(76)))}
#     print(image_points)
#     for point in image_points:
#         # if (point[0] >= 0 and point[0] <= 255) and \
#         #    (point[1] >= 0 and point[1] <= 255):
#             subdivision.insert(point)       
    
#     delaunay = draw_delaunay(correspondance_points['size'], subdivision, pixel_dictionary)

#     return delaunay

# delaunay_triangles = delaunay_triangulation(correspondance_points)

# def affine_transformation(image_rectangle, image_triangle, destination_triangle, size):
#     warp_mat = cv2.getAffineTransform(np.float32(image_triangle), np.float32(destination_triangle))
    
#     destination = cv2.warpAffine(image_rectangle, warp_mat, (size[0], size[1]), None, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)

#     return destination

# def morph_triangle(img_1, img_2, output_image, triangle_1, triangle_2, corresponding_triangle, alpha):
#     rectangle_1, rectangle_2, corresponding_rectangle = cv2.boundingRect(np.float32([triangle_1])), \
#                                                         cv2.boundingRect(np.float32([triangle_2])), \
#                                                         cv2.boundingRect(np.float32([corresponding_triangle]))
    
#     triangle_1_rectangle = []
#     triangle_2_rectangle = []
#     corresponding_triangle_rectangle = []

#     for i in range(3):
#         triangle_1_rectangle.append(((triangle_1[i][0] - rectangle_1[0]), (triangle_1[i][1] - rectangle_1[1])))
#         triangle_2_rectangle.append(((triangle_2[i][0] - rectangle_2[0]), (triangle_2[i][1] - rectangle_2[1])))
#         corresponding_triangle_rectangle.append(((corresponding_triangle[i][0] - corresponding_rectangle[0]),\
#                                                 (corresponding_triangle[i][1] - corresponding_rectangle[1])))

#     mask = np.ones((corresponding_rectangle[3], corresponding_rectangle[2], 3), dtype=np.float32)
#     cv2.fillConvexPoly(mask, np.int32(corresponding_triangle_rectangle), (1.0, 1.0, 1.0), 16, 0)

#     img_1_rectangle = img_1[rectangle_1[1]:rectangle_1[1] + rectangle_1[3], rectangle_1[0]:rectangle_1[0] + rectangle_1[2]]
#     img_2_rectangle = img_2[rectangle_2[1]:rectangle_2[1] + rectangle_2[3], rectangle_2[0]:rectangle_2[0] + rectangle_2[2]]

#     size = (corresponding_rectangle[2], corresponding_rectangle[3])

#     warp_image_1 = affine_transformation(img_1_rectangle, triangle_1_rectangle, corresponding_triangle_rectangle, size)
#     warp_image_2 = affine_transformation(img_2_rectangle, triangle_2_rectangle, corresponding_triangle_rectangle, size)

#     image_rectangle = (1.0 - alpha) * warp_image_1 + alpha * warp_image_2

#     output_image[corresponding_rectangle[1]:corresponding_rectangle[1] + corresponding_rectangle[3], \
#                 corresponding_rectangle[0]:corresponding_rectangle[0] + corresponding_rectangle[2]] = \
#                     output_image[corresponding_rectangle[1]:corresponding_rectangle[1] + corresponding_rectangle[3], \
#                         corresponding_rectangle[0]:corresponding_rectangle[0]+corresponding_rectangle[2]] * ( 1 - mask ) + image_rectangle * mask
    


# def generate_morphed_image(img_1, img_2, landmarks, delaunay_triangles, alpha=0.5):
#     face_1, face_2 = np.float32(img_1), np.float32(img_2)
#     corresponding_points = []
#     first_image_points = landmarks['first']
#     second_image_points = landmarks['second']

#     for i in range(len(first_image_points)):
#         x = (1 - alpha) * first_image_points[i][0] + alpha*second_image_points[i][0]
#         y = (1 - alpha) * first_image_points[i][1] + alpha*second_image_points[i][1]
#         corresponding_points.append((x, y))
    
#     morphed_frame = np.ones(face_1.shape, dtype=face_1.dtype)
    
#     for triangle in delaunay_triangles:
#         a = int(triangle[0])
#         b = int(triangle[1])
#         c = int(triangle[2])
        
#         first_image_triangle = [first_image_points[a], first_image_points[b], first_image_points[c]]
#         second_image_triangle = [second_image_points[a], second_image_points[b], second_image_points[c]]
#         corresponding_triangle = [corresponding_points[a], corresponding_points[b], corresponding_points[c]]
        
#         morph_triangle(face_1, face_2, morphed_frame, first_image_triangle, second_image_triangle, corresponding_triangle, alpha)

#         corresponding_triangle_point_1 = (int(corresponding_triangle[0][0]), int(corresponding_triangle[0][1]))
#         corresponding_triangle_point_2 = (int(corresponding_triangle[1][0]), int(corresponding_triangle[1][1]))
#         corresponding_triangle_point_3 = (int(corresponding_triangle[2][0]), int(corresponding_triangle[2][1]))

#         cv2.line(morphed_frame, corresponding_triangle_point_1, corresponding_triangle_point_2, (255, 255, 255), 1, 8, 0)
#         cv2.line(morphed_frame, corresponding_triangle_point_2, corresponding_triangle_point_3, (255, 255, 255), 1, 8, 0)
#         cv2.line(morphed_frame, corresponding_triangle_point_3, corresponding_triangle_point_1, (255, 255, 255), 1, 8, 0)

#     morphed_image = Image.fromarray(cv2.cvtColor(np.uint8(morphed_frame), cv2.COLOR_BGR2RGB))
#     morphed_image.save('./output/morfadasso','JPEG')

# generate_morphed_image(face_1, face_2, correspondance_points, delaunay_triangles)