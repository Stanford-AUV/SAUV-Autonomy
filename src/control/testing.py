import math

def rotation_direction(current, desired):
    # Normalize angles to the range [-pi, pi]
    current = (current + math.pi) % (2 * math.pi) - math.pi
    desired = (desired + math.pi) % (2 * math.pi) - math.pi
    
    # Calculate the difference
    diff = desired - current
    print(f"diff: {diff}")
    
    # Adjust the difference to be within [-pi, pi]
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.p
    
    # Determine the sign
    return 1 if diff > 0 else -1

# Test cases
print(rotation_direction(2, 1))     # Expected output: 1 (positive, clockwise)
print(rotation_direction(-3, 3))    # Expected output: -1 (negative, counterclockwise)
print(rotation_direction(-1, 1))    # Expected output: 1 (positive, clockwise)
print(rotation_direction(3, -3)) # 1
print(rotation_direction(3, -2)) # 1
