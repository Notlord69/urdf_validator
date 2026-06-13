import numpy as np
import pytest

from urdf_validator_main.physics.geometry_physics import estimate_inertia


# ---------------------------------------------------------------------------
# Sphere: I = 2/5 * m * r^2 * I_3x3
# ---------------------------------------------------------------------------

def test_solid_sphere_unit_mass_unit_radius():
    tensor, conf = estimate_inertia("sphere", [1.0], 1.0)
    expected = np.diag([0.4, 0.4, 0.4])
    np.testing.assert_array_almost_equal(tensor, expected, decimal=6)


def test_solid_sphere_arbitrary_mass_and_radius():
    # m=2, r=3 → I = 2/5 * 2 * 9 = 7.2 on each diagonal
    tensor, conf = estimate_inertia("sphere", [3.0], 2.0)
    expected = np.diag([7.2, 7.2, 7.2])
    np.testing.assert_array_almost_equal(tensor, expected, decimal=6)


def test_sphere_confidence_is_estimated():
    _, conf = estimate_inertia("sphere", [1.0], 1.0)
    assert conf == "estimated"


# ---------------------------------------------------------------------------
# Box: Ixx = m/12*(w²+h²), Iyy = m/12*(l²+h²), Izz = m/12*(l²+w²)
# ---------------------------------------------------------------------------

def test_solid_box_unit_cube():
    # m=12, l=w=h=1 → Ixx=Iyy=Izz = 12/12*(1+1) = 2.0
    tensor, conf = estimate_inertia("box", [1.0, 1.0, 1.0], 12.0)
    expected = np.diag([2.0, 2.0, 2.0])
    np.testing.assert_array_almost_equal(tensor, expected, decimal=6)


def test_solid_box_asymmetric():
    # m=12, l=1, w=2, h=3
    # Ixx = 12/12*(4+9) = 13.0
    # Iyy = 12/12*(1+9) = 10.0
    # Izz = 12/12*(1+4) = 5.0
    tensor, conf = estimate_inertia("box", [1.0, 2.0, 3.0], 12.0)
    expected = np.diag([13.0, 10.0, 5.0])
    np.testing.assert_array_almost_equal(tensor, expected, decimal=6)


def test_box_confidence_is_estimated():
    _, conf = estimate_inertia("box", [1.0, 1.0, 1.0], 1.0)
    assert conf == "estimated"


def test_box_tensor_is_diagonal():
    tensor, _ = estimate_inertia("box", [1.0, 2.0, 3.0], 5.0)
    off_diag = tensor - np.diag(np.diagonal(tensor))
    np.testing.assert_array_almost_equal(off_diag, np.zeros((3, 3)), decimal=12)


# ---------------------------------------------------------------------------
# Cylinder: Ixx=Iyy = m/12*(3r²+L²), Izz = m/2*r²
# ---------------------------------------------------------------------------

def test_solid_cylinder_unit_radius_unit_length():
    # m=1, r=1, L=1 → Ixx=Iyy = 1/12*(3+1)=1/3, Izz = 1/2
    tensor, conf = estimate_inertia("cylinder", [1.0, 1.0], 1.0)
    expected_ixx = 1.0 / 3.0
    expected_izz = 0.5
    np.testing.assert_almost_equal(tensor[0, 0], expected_ixx, decimal=6)
    np.testing.assert_almost_equal(tensor[1, 1], expected_ixx, decimal=6)
    np.testing.assert_almost_equal(tensor[2, 2], expected_izz, decimal=6)


def test_solid_cylinder_ixx_equals_iyy():
    tensor, _ = estimate_inertia("cylinder", [2.0, 3.0], 5.0)
    np.testing.assert_almost_equal(tensor[0, 0], tensor[1, 1], decimal=12)


def test_solid_cylinder_arbitrary():
    # m=12, r=2, L=3 → Ixx=Iyy = 12/12*(12+9) = 21, Izz = 12/2*4 = 24
    tensor, conf = estimate_inertia("cylinder", [2.0, 3.0], 12.0)
    np.testing.assert_almost_equal(tensor[0, 0], 21.0, decimal=6)
    np.testing.assert_almost_equal(tensor[2, 2], 24.0, decimal=6)


def test_cylinder_confidence_is_estimated():
    _, conf = estimate_inertia("cylinder", [1.0, 1.0], 1.0)
    assert conf == "estimated"


# ---------------------------------------------------------------------------
# Mesh fallback
# ---------------------------------------------------------------------------

def test_mesh_returns_none_tensor():
    tensor, conf = estimate_inertia("mesh", None, 1.0)
    assert tensor is None


def test_mesh_confidence_is_guessed():
    _, conf = estimate_inertia("mesh", None, 1.0)
    assert conf == "guessed"


# ---------------------------------------------------------------------------
# Unknown geometry type
# ---------------------------------------------------------------------------

def test_unknown_type_returns_none():
    tensor, conf = estimate_inertia("unknown_shape", None, 1.0)
    assert tensor is None
    assert conf == "missing"


# ---------------------------------------------------------------------------
# Zero mass
# ---------------------------------------------------------------------------

def test_zero_mass_sphere_returns_zero_tensor():
    tensor, conf = estimate_inertia("sphere", [1.0], 0.0)
    np.testing.assert_array_almost_equal(tensor, np.zeros((3, 3)), decimal=12)
    assert conf == "estimated"


# ---------------------------------------------------------------------------
# No-crash guarantee — bad inputs must not raise
# ---------------------------------------------------------------------------

def test_none_dims_for_box_does_not_raise():
    tensor, conf = estimate_inertia("box", None, 1.0)
    assert tensor is None
    assert conf == "missing"


def test_wrong_dim_count_does_not_raise():
    tensor, conf = estimate_inertia("sphere", [], 1.0)
    assert tensor is None
    assert conf == "missing"
