import pinocchio as pin

# Load model - update this path to your MJCF file
model_path = "./so101_pinocchio.xml"
pin_model, _, _ = pin.buildModelsFromMJCF(model_path)
pin_data = pin_model.createData()

print(f"Number of frames: {pin_model.nframes}")
print(f"Size of oMf: {len(pin_data.oMf)}")
print(f"Number of joints: {pin_model.njoints}")
print(f"Number of DOF (nq): {pin_model.nq}")
print()

# Check camera_front frame ID
site_names = ["shoulderSite", "elbowSite", "wristSite", "EE", "camera_wrist", "camera_front"]
print("Checking site frame IDs:")
for site_name in site_names:
    try:
        frame_id = pin_model.getFrameId(site_name)
        frame = pin_model.frames[frame_id]
        print(f"  {site_name:20s} -> frame_id={frame_id:2d} type={frame.type}")
        if frame_id >= len(pin_data.oMf):
            print(f"    WARNING: frame_id {frame_id} >= oMf size {len(pin_data.oMf)}")
    except Exception as e:
        print(f"  {site_name:20s} -> ERROR: {e}")
print()

# List all frames
print("All frames:")
for i in range(pin_model.nframes):
    frame = pin_model.frames[i]
    print(f"  [{i:2d}] {frame.name:30s} type={frame.type} parent={frame.parent}")

print()

# Try to find camera_front
print("Looking for 'camera_front':")
found_site = False
for i in range(pin_model.nframes):
    frame = pin_model.frames[i]
    if frame.name == "camera_front":
        found_site = True
        print(f"  [{i:2d}] FOUND! {frame.name:30s} type={frame.type}")
    if "camera" in frame.name.lower():
        print(f"  [{i:2d}] {frame.name:30s} type={frame.type}")

if not found_site:
    print("  WARNING: 'camera_front' site not found in Pinocchio frames!")
    print("  This means Pinocchio's MJCF parser didn't import the site.")
