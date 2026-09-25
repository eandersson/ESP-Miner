import { Component, ChangeDetectionStrategy } from '@angular/core';
import { LayoutService } from "./service/app.layout.service";

@Component({
    selector: 'app-footer',
    templateUrl: './app.footer.component.html',
    changeDetection: ChangeDetectionStrategy.Eager,
    standalone: false
})
export class AppFooterComponent {
    constructor(public layoutService: LayoutService) { }
}
