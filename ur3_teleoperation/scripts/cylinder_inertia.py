import math

def CylinderInertia(mass, diameter, lenght):
    Ixy = (mass/12)*(3*(diameter/2)**2 + lenght**2)
    Iz = (mass * (diameter/2)**2 )/2

    return Ixy, Iz

pesa_acero = [0.57, 0.06358, 0.023]
pesa_uhm = [0.032, 0.04118, 0.02868]
pesa_3d = [0.038, 0.0494, 0.06956]
destornillador = [0.071, 0.03978, 0.208]

acero_xy, acero_z = CylinderInertia(pesa_acero[0],pesa_acero[1],pesa_acero[2])
uhm_xy, uhm_z = CylinderInertia(pesa_uhm[0],pesa_uhm[1],pesa_uhm[2])
d3_xy, d3_z = CylinderInertia(pesa_3d[0],pesa_3d[1],pesa_3d[2])
dest_xy, dest_z = CylinderInertia(destornillador[0],destornillador[1],destornillador[2])

print("Pesa de acero, \n    Ixy:", acero_xy, "\n    Iz:", acero_z,
    "\n    Radius:", pesa_acero[1]/2)
print("Pesa de UHM, \n    Ixy:", uhm_xy,"\n    Iz:", uhm_z,
    "\n    Radius", pesa_uhm[1]/2)
print("Pesa 3D, \n    Ixy:", d3_xy,"\n    Iz:", d3_z,
    "\n    Radius:", pesa_3d[1]/2)
print("Destornillador, \n    Ixy:", dest_xy,"\n    Iz:", dest_z,
    "\n    Radius:", destornillador[1]/2)