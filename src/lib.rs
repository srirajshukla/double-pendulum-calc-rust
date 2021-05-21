use std::fmt::Formatter;
mod tests;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Point {
    /// Start a new point with default values.
    pub fn new_def() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
    /// Function to make a new instance of Point struct
    /// with values x and y of type f64
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }

    pub fn location(&mut self, x: f64, y: f64) -> () {
        self.x = x;
        self.y = y;
    }

    pub fn x(&self) -> f64{
        self.x
    }
    pub fn y(&self) -> f64{
        self.y
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DoublePendulam {
    m1: f64,
    m2: f64,
    a1: f64,
    a2: f64,
    l1: f64,
    l2: f64,
    c1: Point,
    c2: Point,
    v1: f64,
    v2: f64,
    acc1: f64,
    acc2: f64,
    g: f64,
    damp_factor: f64,
}

impl DoublePendulam {
    pub fn get_joint(&self) -> Point{
        Point::new(self.c1.x, self.c1.y)
    }

    pub fn get_end(&self) -> Point{
        Point::new(self.c2.x, self.c2.y)
    }

    fn calc_pos(&mut self) {
        self.c1.x = self.l1 * self.a1.sin();
        self.c1.y = -1.0*self.l1 * self.a1.cos();
        self.c2.x = self.c1.x + self.l2 * self.a2.sin();
        self.c2.y = self.c1.y - self.l2 * self.a2.cos();
    }

    fn new_acc(&mut self) {
        let p1 = -self.g*(2.0*self.m1 + self.m2)*self.a1.sin();
        let p2 = -self.m2*self.g*(self.a1 - 2.0*self.a2).sin();
        let p3 = -2.0*(self.a1 - self.a2).sin()*self.m2;
        let p4 = self.v2*self.v2*self.l2 + self.v1*self.v1*(self.a1 - self.a2).cos();
        let p5 = 2.0*self.m1 + self.m2 - self.m2*(2.0*self.a1 - 2.0*self.a2).cos();
        
        self.acc1 = (p1 + p2 + p3*p4) / (self.l1*p5);

        let p1 = 2.0*(self.a1 - self.a2).sin();
        let p2 = self.v1*self.v1*self.l1 *(self.m1 + self.m2);
        let p3 = self.g*(self.m1 + self.m2) * self.a1.cos();
        let p4 = self.v2*self.v2*self.l2*self.m2*(self.a1 - self.a2).cos();

        self.acc2 = p1*(p2 + p3 + p4)/(self.l2*p5);
    }

    fn new_angle(&mut self){
        self.a1 += self.v1;
        self.a2 += self.v2;
    }

    fn new_vel(&mut self){
        self.v1 += 0.2*self.acc1;
        self.v2 += 0.2*self.acc2;

        // todo: add a parameter and setter method for dampening constant. 
        self.v1 *= 1.0 - self.damp_factor;
        self.v2 *= 1.0 - self.damp_factor;
    }

    pub fn new_pos(&mut self){
        self.new_vel();
        self.new_acc();
        self.new_angle();
        self.calc_pos();
    }

    pub fn new(m1: f64, m2: f64, a1: f64, a2: f64, l1: f64, l2: f64, damp_factor: f64) -> Self {
        let mut obj = DoublePendulam {
            m1,
            m2,
            a1,
            a2,
            l1,
            l2,
            c1: Point::new(0.0, 0.0),
            c2: Point::new(0.0, 0.0),
            v1: 0.0,
            v2: 0.0,
            acc1: 0.0,
            acc2: 0.0,
            g: 1.0,
            damp_factor,
        };
        obj.calc_pos();
        return obj;
    }

    pub fn m1(&self) -> f64{
        return self.m1;
    }
    pub fn m2(&self) -> f64{
        return self.m2;
    }
    pub fn l1(&self) -> f64{
        return self.l1;
    }
    pub fn l2(&self) -> f64{
        return self.l2;
    }
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}
