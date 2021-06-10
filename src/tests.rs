#[cfg(test)]
mod tests {
    use crate::Point;
    use crate::DoublePendulum;
    #[test]
    fn dp (){
        let pi = std::f64::consts::PI;
        let p = DoublePendulum::new(10.0, 10.0, 0.0, pi/2.0, 50.0, 50.0, 0.01);

        let joint = p.get_joint();
        assert_eq!(joint.x, 0.0);
        assert_eq!(joint.y, -50.0);
    }
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn new_point(){
        let p = Point::new(3.0, 4.0);
        assert_eq!(p.x, 3.0);
        assert_eq!(p.y, 4.0);
    }

    #[test]
    fn point_location_change() {
        let mut p = Point::new_def();
        p.location(2.0, 3.0);
        assert_eq!(p.x, 2.0);
        assert_eq!(p.y, 3.0);
    }
}
