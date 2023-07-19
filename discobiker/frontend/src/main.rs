use leptos::{error::Result, *};
use thiserror::Error;
use web_sys;
use js_sys::Array;
use wasm_bindgen_futures::JsFuture;

#[derive(Error, Clone, Debug)]
pub enum BluetoothError {
    #[error("WebBluetooth is not supported on this browser")]
    Unsupported,
    #[error("Device was not found")]
    NotFound(String),
    #[error("failed to request device: {0:?}")]
    RequestDevice(String),
}

async fn list_devices() -> Result<web_sys::BluetoothDevice> {
    let nav = window().navigator();
    let bluetooth = nav.bluetooth().ok_or(BluetoothError::Unsupported)?;
    let mut options = web_sys::RequestDeviceOptions::new();
    let mut filter = web_sys::BluetoothLeScanFilterInit::new();
    filter.name("DiscobikeR");
    let filters = Array::new();
    filters.push(&filter.into());
    options.filters(&filters.into());
    let device: web_sys::BluetoothDevice = JsFuture::from(bluetooth.request_device(&options))
        .await
        .map_err(|e| BluetoothError::RequestDevice(format!("{:?}", e)))?
        .into();
    Ok(device)
}

#[component]
fn Device(
    cx: Scope,
    #[prop(into)]
    device: MaybeSignal<web_sys::BluetoothDevice>
) -> impl IntoView {
    view!{ cx,
        <ul>
            <li>"Name: " {move || device.get().name()}</li>
        </ul>
    }
}

#[component]
fn App(cx: Scope) -> impl IntoView {
    let fallback = move |cx, errors: RwSignal<Errors>| {
        let error_list = move || {
            errors.with(|errors| {
                errors
                    .iter()
                    .map(|(_, e)| view! { cx, <li>{e.to_string()}</li> })
                    .collect_view(cx)
            })
        };

        view! { cx,
            <div class="error">
                <h2>"Error"</h2>
                <ul>{error_list}</ul>
            </div>
        }
    };

    let connect = create_action(cx, |_| list_devices());
    view! { cx,
        <p>"Bluetooth!"</p>
        <button
            on:click=move |_| connect.dispatch(())
        >Connect</button>
        <ErrorBoundary fallback>
            {move || connect.value().get().map(|dev| dev.map(|dev| view! { cx, <Device device=dev />}))}
        </ErrorBoundary>
    }
}

fn main() {
    _ = console_log::init_with_level(log::Level::Debug);
    console_error_panic_hook::set_once();
    mount_to_body(|cx| view! { cx,  <App /> })
}