use crate::simdata_mod::SimulationData;
use plotters::backend::BitMapBackend;
use plotters::chart::ChartBuilder;
use plotters::coord::types::{RangedCoordf32, RangedCoordf64};
use plotters::coord::Shift;
use plotters::element::{PathElement, PointCollection};
use plotters::prelude::{
    Cartesian2d, ChartContext, Color, CoordTranslate, DrawingArea, DrawingBackend, DynElement,
    IntoDrawingArea, IntoFont, LineSeries, RGBColor, BLACK, BLUE, RED, WHITE,
};

pub(crate) fn make_line_plot<'a, const L: usize>(
    file_name: &'a str,
    plot_title: &'a str,
    x_label: &'a str,
    x_var: usize,
    y_label: &'a str,
    y_var: usize,
    line_series_label: &'a str,
    line_color: &'a RGBColor,
    xdims: [f32; 2],
    ydims: [f32; 2],
    data: &SimulationData<L>,
) -> Result<
    (
        ChartContext<'a, BitMapBackend<'a>, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
        DrawingArea<BitMapBackend<'a>, Shift>,
    ),
    Box<dyn std::error::Error>,
> {
    let xmin = xdims[0];
    let xmax = xdims[1];

    let ymin = ydims[0];
    let ymax = ydims[1];

    let len = data.len as usize;

    let root = BitMapBackend::new(file_name, (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(plot_title, ("sans-serif", 50).into_font())
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(xmin..xmax, ymin..ymax)?;

    chart
        .configure_mesh()
        .x_desc(x_label)
        .y_desc(y_label)
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

    // ########## The number in ~~~.col( x ).~~~~ is what determines which variable we are
    // ########## looking at. Since this is state-dependant, I think it would be nice to get a
    // ########## string or otherwise more general way of specifying that. But this works for now.

    chart
        .draw_series(LineSeries::new(
            (0..len).map(|ind| {
                (
                    data.get_val(ind as usize, x_var) as f32,
                    data.get_val(ind as usize, y_var) as f32,
                )
            }),
            line_color.stroke_width(2),
        ))?
        .label(line_series_label)
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], line_color));
/*
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .expect("Uh Oh: panic message");


 */
   // root.present()
   //     .expect("Error: no panic message. Try rm -rfv /");

    Ok((chart, root))
}

pub(crate) fn add_line_to_plot<'a, const L: usize, T: DrawingBackend, TT: CoordTranslate>(
    x_var: usize,
    y_var: usize,
    line_series_label: &'a str,
    line_color: &'a RGBColor,
    data: &SimulationData<L>,
    mut chart: &mut ChartContext<'a, T, TT>,
) -> Result<(), Box<dyn std::error::Error>>
where
    for<'b> &'b DynElement<'static, T, (f32, f32)>:
        PointCollection<'b, <TT as CoordTranslate>::From>, <T as DrawingBackend>::ErrorType: 'static
{
    let len = data.len as usize;

    chart
        .draw_series(LineSeries::new(
            (0..len).map(|ind| {
                (
                    data.get_val(ind, x_var) as f32,
                    data.get_val(ind, y_var) as f32,
                )
            }),
            line_color.stroke_width(2),
        ))?
        .label(line_series_label)
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], line_color));

    Ok(())

}

pub(crate) fn plot_plot<'a, T: DrawingBackend + 'a, TT: CoordTranslate>(
    mut chart: &mut ChartContext<'a, T, TT>,
    root: &DrawingArea<T, Shift>,
) -> () {
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .expect("Uh Oh: panic message");

    root.present()
        .expect("Error: no panic message. Try rm -rfv /");
}
