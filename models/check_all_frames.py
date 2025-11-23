import pinocchio as pin

# Load model - update this path to your MJCF file
model_path = "./so101_pinocchio.xml"
pin_model, _, _ = pin.buildModelsFromMJCF(model_path)

print("ALL FRAMES IN MODEL:")
print(f"{'ID':<4} {'Name':<30} {'Type':<20} {'ParentJoint':<15}")
print("=" * 70)
for i in range(pin_model.nframes):
    frame = pin_model.frames[i]
    print(f"{i:<4} {frame.name:<30} {str(frame.type):<20} {frame.parentJoint:<15}")
